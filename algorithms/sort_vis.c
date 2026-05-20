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
 * Study alongside: procedural/generational/maze.c — both turn an
 *   iterative algorithm (carve a maze / sort an array) into a state
 *   machine that emits one visible event per tick, so the animation
 *   IS the algorithm.
 *
 * Section map:
 *   §1 config  — N_ELEMS, HUD_ROWS, frame timing, color pair IDs, phase enums
 *   §2 clock   — monotonic timer + nanosleep
 *   §3 color   — bar/compare/swap/sort + HUD/HINT pairs
 *   §4 types   — Scene + RenderHints + per-algorithm State bundles
 *   §5 sort    — five state-machine sorters, one operation per call
 *   §7 scene   — bar geometry + draw_bars + HUD formatting
 *   §8 app     — signals, resize, input, frame loop, main
 *
 * Keys:  q/ESC quit   TAB next alg   space scramble   p pause   +/- speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra algorithms/sort_vis.c \
 *       -o sort_vis -lncurses -lm
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
 * Data-structure : A single Scene struct owns the int array arr[N_ELEMS]
 *                  plus an AlgState bundle of FIVE per-algorithm cursor
 *                  states (BubbleState, InsertState, SelectState,
 *                  QuickState, HeapState).  Each *_step function reads /
 *                  writes only its own State block — the other four lie
 *                  dormant until a TAB switch + scramble reseeds them.
 *                  QuickState carries an explicit lo[]/hi[] range stack
 *                  so quicksort never recurses — every algorithm can be
 *                  paused and resumed mid-step.
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
 * References
 * ──────────
 *   ── Original algorithm papers ───────────────────────────────────
 *   [1] Hoare, C. A. R. (1962), "Quicksort", The Computer Journal
 *       5(1), pp. 10-16 — the ORIGINAL Quicksort paper.  Defines
 *       the partition + recurse structure.  Short and historically
 *       important; reads in under 30 minutes.
 *   [2] Williams, J. W. J. (1964), "Algorithm 232: Heapsort",
 *       Comm. ACM 7(6), pp. 347-348 — the ORIGINAL Heapsort paper.
 *       Introduces the binary-heap data structure simultaneously
 *       with its use as a sorting tool.
 *   [3] Sedgewick, R. (1978), "Implementing Quicksort programs",
 *       Comm. ACM 21(10), pp. 847-857 — the classic optimisation
 *       study.  Covers pivot choice, small-array switch to insertion
 *       sort, recursion-elimination via explicit stack (the exact
 *       trick quick_step uses in §5).
 *
 *   ── Canonical textbooks ────────────────────────────────────────
 *   [4] Knuth, D. E. (1998), "The Art of Computer Programming,
 *       Vol. 3: Sorting and Searching" (2nd ed.), Addison-Wesley —
 *       the encyclopaedic reference.  §5.2.2 (exchange sorts),
 *       §5.2.1 (insertion), §5.2.3 (selection + heapsort),
 *       §5.2.2 (quicksort) cover all five algorithms in depth.
 *   [5] Sedgewick, R. & Wayne, K. (2011), "Algorithms" (4th ed.),
 *       Addison-Wesley — Chs. 2.1-2.4 cover bubble, insertion,
 *       selection, quick, heap.  The most practical introduction;
 *       Java code online at algs4.cs.princeton.edu.
 *   [6] Cormen, T. H., Leiserson, C. E., Rivest, R. L. & Stein, C.
 *       (2022), "Introduction to Algorithms" (4th ed.), MIT Press —
 *       Ch. 6 (heapsort with the sift-down proof), Ch. 7 (quicksort
 *       average-case analysis).  Rigorous proofs of the O(N log N)
 *       bounds.
 *
 *   ── Coroutines / state machines (the §5 step-function pattern) ──
 *   [7] Knuth, D. E. (1997), "The Art of Computer Programming,
 *       Vol. 1: Fundamental Algorithms" (3rd ed.), Addison-Wesley —
 *       §1.4.2 "Coroutines" formalises the resumable-procedure
 *       pattern that each *_step() function implements via static
 *       cursor state.
 *   [8] Conway, M. E. (1963), "Design of a separable transition-
 *       diagram compiler", Comm. ACM 6(7), pp. 396-408 — coined the
 *       term "coroutine" and gave the first formal definition.
 *       Historical interest; the ancestor of the generator / yield
 *       idiom modern languages provide.
 *
 *   ── Algorithm visualisation ────────────────────────────────────
 *   [9] Bostock, M., "Visualizing Algorithms" —
 *       https://bost.ocks.org/mike/algorithms/ — the modern essay on
 *       *why* animated algorithms aid intuition.  Inspiration for
 *       this file's one-operation-per-frame design.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [10] https://en.wikipedia.org/wiki/Sorting_algorithm — side-by-side
 *       comparison table of every named sort with complexity bounds,
 *       stability flags, and links to per-algorithm pages with
 *       animations.
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
 *   quick_step():  Lomuto, iterative.  Maintains explicit stack of (lo, hi).
 *     1. SCAN phase: scan j over [lo, hi-1] comparing arr[j] vs pivot=arr[hi];
 *        if arr[j] <= pivot, swap into the "≤ pivot" prefix (i++; swap i,j).
 *     2. FINALIZE: place pivot at arr[i+1]; push left and right sub-ranges.
 *     3. POP phase: pop next valid range, prime new partition; iterate.
 *     4. Done when stack is empty.
 *
 *   heap_step():
 *     1. BUILD phase: Floyd's O(N) heapify — sift-down from N/2-1 down to 0.
 *     2. EXTRACT phase: swap arr[0] with arr[n-1]; n--; sift-down new root.
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
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first as prose.
 *      Companion file: procedural/generational/maze.c uses the same
 *      iterative-coroutine pattern for maze carving / solving.
 *   2. §4 data types — Scene, AlgState, the five per-algorithm State
 *      structs.  EVERY function takes Scene*; understanding the shape
 *      of Scene unlocks the rest of the file.
 *   3. §5 sort — THE HEART of this file.  FIVE step functions, one
 *      per algorithm.  Each step does ONE compare-or-swap and yields:
 *        bubble_step   — adjacent compare on (j, j+1)
 *        insert_step   — slide-left until in place
 *        select_step   — running argmin, then one emit-swap
 *        quick_step    — Lomuto partition with explicit range stack
 *        heap_step     — sift-down phases (build then extract)
 *      Each step body is 4–8 lines of named-helper calls; the helpers
 *      that implement each named step sit just above their step fn.
 *   4. §7 scene — bar chart renderer + HUD.  draw_bar splits into
 *      geometry helpers (bar_value_to_cell_height, bar_column_span,
 *      paint_bar_column); draw_hud splits into format / top / bottom.
 *   5. §1-§3, §8 — config / clock / colour / app loop.  main() reads
 *      as: signals_install → terminal_init → scene_init → frame loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Almost everything lives in `Scene sc`, passed by pointer everywhere.
 *   The only file-scope state is the POSIX signal flags (g_quit /
 *   g_resize) because handlers can't receive a context pointer.
 *
 *   sc->arr[N_ELEMS]      the array being sorted (1..N when freshly
 *                         scrambled; always a permutation).
 *   sc->alg               current algorithm enum (ALG_BUBBLE..ALG_HEAP).
 *   sc->done              bool — has the current algorithm finished?
 *   sc->cmp_count,
 *   sc->swp_count         per-run statistics (HUD shows these).
 *   sc->hints.cmp1/cmp2   indices painted GOLD this frame (compare).
 *   sc->hints.swp1/swp2   indices painted RED-BOLD (just swapped).
 *                         −1 in any slot means "no glow there".
 *   sc->paused, sc->speed UI controls (toggled / adjusted by handle_input).
 *   sc->rows, sc->cols    terminal extent for the current frame.
 *
 *   PER-ALGORITHM CURSOR STATES (all inside sc->algs.<name>):
 *     sc->algs.bubble.{i, j}                    — pass + pair cursor
 *     sc->algs.insert.{i, j}                    — outer + slide cursor
 *     sc->algs.select.{i, j, min}               — boundary + scan + argmin
 *     sc->algs.quick.{lo[], hi[], top,          — explicit range stack
 *                     cur_lo, cur_hi, i, j, phase}    + Lomuto cursors
 *     sc->algs.heap.{n, sift_from, phase}       — heap size + sift cursor
 *
 *   PHASE ENUMS:
 *     QS_PHASE_SCAN / QS_PHASE_POP        — quicksort coroutine phases
 *     HEAP_PHASE_BUILD / HEAP_PHASE_EXTRACT — heapsort coroutine phases
 *
 * Background you need
 * ───────────────────
 *   - Comparison-sort intuition (you've likely written bubble sort).
 *   - O-notation: bubble/insertion/selection are O(N²); heap/quick
 *     are O(N log N).
 *   - Iterative state machines — converting a recursive function
 *     into a step-by-step state-driven loop.  This is THE technique
 *     that makes algorithm visualization possible.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Coroutines / fibers / generators (Python yield, C++ co_yield).
 *     We HAND-ROLL coroutines using struct state.  Same idea, no
 *     language support needed.
 *   - Stable-sort theory.  Brief mention in CONCEPTS; not essential.
 *   - External / parallel / non-comparison sorts (radix, merge,
 *     timsort).  Five comparison sorts are the canonical pedagogy set.
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

/* Quicksort coroutine phases — replaces magic 0/1 inside QuickState.phase. */
enum { QS_PHASE_SCAN = 0, QS_PHASE_POP = 1 };

/* Heapsort coroutine phases — replaces magic 0/1 inside HeapState.phase. */
enum { HEAP_PHASE_BUILD = 0, HEAP_PHASE_EXTRACT = 1 };

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
/* §4  data types — RenderHints, per-algorithm states, Scene              */
/* ===================================================================== */

/*
 *  Layered ownership in this file:
 *
 *     Scene
 *       ├── arr[]               (data being sorted)
 *       ├── alg                 (which algorithm is running)
 *       ├── AlgState algs       ← bundle of per-algorithm cursors
 *       │     ├── BubbleState   ← iterative coroutine state for each
 *       │     ├── InsertState     of the five sorts.  Exactly one of
 *       │     ├── SelectState     these matches `alg` and is "active".
 *       │     ├── QuickState
 *       │     └── HeapState
 *       ├── RenderHints hints   ← what to highlight THIS frame
 *       ├── counters            (compares, swaps — for HUD)
 *       └── UI flags + extent   (paused, speed, rows, cols)
 *
 *  Every persistent value the program needs is reachable from a single
 *  Scene*.  No file-scope simulation state exists; only the two POSIX
 *  signal flags (g_quit, g_resize) remain global because signal handlers
 *  cannot receive a context pointer.
 */

/*
 * RenderHints — per-tick visual cues from the sort to the renderer.
 *
 * Intent
 *   Every step function sets these BEFORE returning, and the renderer
 *   reads them on the next frame to colour the right bars:
 *     cmp1, cmp2 → gold "comparing now"      (PAIR_CMP)
 *     swp1, swp2 → red bold "just swapped"   (PAIR_SWP)
 *   Sentinel −1 means "no glow this frame for that slot".
 *
 * Why a SEPARATE struct (not inline fields in Scene)
 *   Groups four ints that always move together — "what is the algorithm
 *   doing visibly this tick?".  Reset to −1 on every step entry; written
 *   only when a compare or swap actually happens.  Naming them as a
 *   bundle keeps the bar_pair lookup readable (`if (i == h.cmp1)`).
 *
 * Members
 *   cmp1, cmp2   indices being compared this tick; −1 if no compare
 *   swp1, swp2   indices just swapped this tick;   −1 if no swap
 *
 * Invariants
 *   −1 ≤ cmp* ≤ N_ELEMS − 1 and −1 ≤ swp* ≤ N_ELEMS − 1
 *   cmp1 ≠ cmp2  AND  swp1 ≠ swp2   (never two cursors on one index)
 *   A non-(−1) value remains a valid arr[] index until next step entry.
 *
 * References
 *   [9] Bostock — "Visualizing Algorithms" (the colour-pair convention
 *       gold = comparing, red = swapping is borrowed directly).
 */
typedef struct {
    int cmp1, cmp2;   /* gold pair      (−1 = no compare this frame) */
    int swp1, swp2;   /* red-bold pair  (−1 = no swap    this frame) */
} RenderHints;

/*
 * Per-algorithm cursor states — DESIGN NOTE that applies to all five.
 *
 *   Each sort is rewritten here as an ITERATIVE COROUTINE: its natural
 *   recursion or nested loops are flattened into a state machine whose
 *   "instruction pointer" lives in the struct below.  The step function
 *   reads its state, does ONE compare-or-swap, writes the state back,
 *   and returns.  Across frames the cursor walks the same path it would
 *   in a normal recursive / nested-loop implementation, but the user
 *   sees every intermediate move.
 *
 *   Each *State struct therefore answers:
 *     "If we halted the algorithm right now, what variables do we need
 *      to restart from this exact point?"
 *
 *   References
 *     [7] Knuth TAOCP Vol. 1 §1.4.2 — coroutines + resumable execution.
 *     [8] Conway 1963 — the original "Design of a Separable Transition
 *         Diagram Compiler" coroutine paper; this is the same idea
 *         implemented by hand.
 */

/*
 * BubbleState — outer pass index + inner adjacent-pair cursor.
 *
 * Intent
 *   Drive a bubble-sort scan one comparison at a time.  Bubble sort
 *   makes repeated left-to-right sweeps; after pass i, the LARGEST
 *   element of the remaining unsorted prefix has "bubbled" to its
 *   final position at index N − 1 − i.
 *
 * Members
 *   i   pass count, 0..N − 2.  After pass i finishes, suffix
 *       arr[N − 1 − i .. N − 1] is sorted (final values, never touched again).
 *   j   current adjacent-pair cursor inside pass i, scanning 0..N − 2 − i.
 *       Each step compares arr[j] vs arr[j+1] and conditionally swaps.
 *
 * Invariants
 *   0 ≤ i < N_ELEMS  AND  0 ≤ j ≤ N_ELEMS − 2 − i
 *   For all k in [N − i .. N − 1]: arr[k] is in its final sorted position.
 *   Total comparisons across a full run = N(N−1)/2 (worst + average).
 *
 * References
 *   [4] Knuth TAOCP Vol. 3 §5.2.2 — exchange sorting; bubble sort
 *       analysed under "the simplest exchange sort".
 *   [5] Sedgewick & Wayne, Algorithms 4th ed., §2.1 — pedagogical intro;
 *       why bubble sort is the standard "first sort" despite its cost.
 */
typedef struct {
    int i, j;
} BubbleState;

/*
 * InsertState — outer cursor + slide-left cursor.
 *
 * Intent
 *   Drive insertion sort one slide-comparison at a time.  Insertion
 *   sort grows a sorted prefix by picking the next unsorted element
 *   and walking it left past every strictly-greater predecessor.
 *
 * Members
 *   i   next unsorted index to insert, 1..N − 1.  Prefix arr[0..i − 1]
 *       is sorted as an invariant when each new i is picked up.
 *   j   slide cursor for the element being inserted.  Starts at i, then
 *       decrements as long as arr[j − 1] > arr[j], swapping each step.
 *       Stops when arr[j − 1] ≤ arr[j] OR j == 0.
 *
 * Invariants
 *   1 ≤ i ≤ N_ELEMS  AND  0 ≤ j ≤ i
 *   At the START of each outer iteration: arr[0..i − 1] is sorted.
 *   Best case (already sorted): N − 1 compares total, 0 swaps.
 *   Worst case (reverse sorted): N(N − 1)/2 compares + swaps.
 *
 * References
 *   [4] Knuth TAOCP Vol. 3 §5.2.1 — insertion sorting; the "straight
 *       insertion" baseline against which more complex sorts are judged.
 *   [5] Sedgewick & Wayne §2.1 — why insertion sort is the algorithm of
 *       choice for SMALL N (typically used as quicksort's base case).
 */
typedef struct {
    int i, j;
} InsertState;

/*
 * SelectState — outer cursor + scan cursor + running-min cursor.
 *
 * Intent
 *   Drive selection sort one scan-comparison at a time.  Selection sort
 *   finds the minimum of the unsorted suffix, swaps it into the next
 *   sorted slot, and repeats.  Unlike bubble/insertion it does at most
 *   N − 1 SWAPS in total (one per outer iteration), even when N(N − 1)/2
 *   compares are needed.
 *
 * Members
 *   i     sorted-prefix boundary, 0..N − 1.  arr[0..i − 1] is sorted AND
 *         every value in it is ≤ every value in arr[i..N − 1].
 *   j     scan cursor walking i + 1 .. N − 1 looking for a new minimum.
 *   min   index of the smallest arr[k] seen so far in this pass,
 *         starting at i and updated whenever arr[j] < arr[min].
 *
 * Invariants
 *   0 ≤ i ≤ min ≤ j ≤ N_ELEMS
 *   arr[min] ≤ arr[k] for every k in [i..j − 1]   (running argmin).
 *   Always N(N − 1)/2 compares regardless of input order.
 *   At most N − 1 swaps — minimum write cost of any compare-sort.
 *
 * References
 *   [4] Knuth TAOCP Vol. 3 §5.2.3 — selection-by-counting + tournament
 *       refinements; the lower swap bound makes selection sort the
 *       choice when writes are expensive (e.g. flash memory).
 *   [5] Sedgewick & Wayne §2.1 — selection vs insertion comparison.
 */
typedef struct {
    int i, j, min;
} SelectState;

/*
 * QuickState — Lomuto iterative quicksort with an explicit range stack.
 *
 * Intent
 *   Drive an iterative Lomuto-partition quicksort one inner-loop tick
 *   at a time.  Recursion is replaced by an EXPLICIT stack of pending
 *   sub-ranges (lo[]/hi[]).  At each step we are either advancing the
 *   Lomuto partition over the current sub-range (QS_PHASE_SCAN), or
 *   finalizing the partition and popping the next sub-range (QS_PHASE_POP).
 *
 *   Two-phase per range:
 *     QS_PHASE_SCAN:  j walks cur_lo..cur_hi − 1; if arr[j] ≤ pivot,
 *                     swap arr[++i] ↔ arr[j] (grow the ≤-pivot prefix).
 *     QS_PHASE_POP:   place pivot at i + 1; push the two non-trivial
 *                     sub-ranges; pop the next one and re-enter SCAN.
 *
 * Why an EXPLICIT stack (not C recursion)
 *   • A single recursive call would do the whole sort BEFORE returning,
 *     so the visualisation would never get a chance to draw a frame.
 *   • Pinning the pivot to arr[cur_hi] (Lomuto rather than Hoare) keeps
 *     the partition state expressible in two ints (i, j) + the range
 *     bounds, which is cheap to resume from.
 *
 * Members
 *   lo[QS_STACK], hi[QS_STACK]   pending sub-range bounds
 *   top                          next-free slot in lo[]/hi[];  depth.
 *                                Pushed range is [lo[top], hi[top]].
 *   cur_lo, cur_hi               currently-being-partitioned range.
 *                                Pivot lives at cur_hi.
 *   i, j                         Lomuto cursors:
 *                                  i = last index of the "≤ pivot" prefix
 *                                  j = scan cursor moving cur_lo→cur_hi
 *   phase                        QS_PHASE_SCAN = scanning current range
 *                                QS_PHASE_POP  = finalize pivot + pop next
 *
 * Invariants
 *   0 ≤ top ≤ QS_STACK   (stack never overflows for N ≤ N_ELEMS).
 *   Sub-ranges in lo[]/hi[] are pairwise DISJOINT subsets of [0..N − 1].
 *   In QS_PHASE_SCAN:  cur_lo ≤ i + 1 ≤ j ≤ cur_hi
 *                      arr[cur_lo..i] all ≤ pivot, arr[i+1..j−1] all > pivot.
 *   In QS_PHASE_POP:   partition has just completed for [cur_lo..cur_hi].
 *   Worst-case stack depth O(log N) only if we always push the LARGER
 *   sub-range first (Sedgewick 1978) — we accept O(N) worst case for
 *   pedagogical clarity; QS_STACK = 128 is comfortable for N_ELEMS = 48.
 *
 * References
 *   [1] Hoare 1962 — original "Quicksort" paper, CACM 5(1).
 *   [3] Sedgewick 1978 — "Implementing Quicksort Programs", CACM 21(10):
 *       explicit-stack derivation + tail-call trick to bound depth.
 *   [4] Knuth TAOCP Vol. 3 §5.2.2 — quicksort analysis (the N log N
 *       average case + N² pathological derivation).
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
 * HeapState — max-heap build + extract phases.
 *
 * Intent
 *   Drive heapsort one sift step at a time.  Heapsort runs in two
 *   distinct phases:
 *
 *     HEAP_PHASE_BUILD ("heapify"):  walk sift_from from N/2 − 1 down
 *                                    to 0, calling heap_sift at each
 *                                    start point.  Establishes the
 *                                    max-heap property on arr[0..N − 1].
 *     HEAP_PHASE_EXTRACT:            repeatedly swap arr[0] with
 *                                    arr[n − 1] (extracting the max);
 *                                    n-- ; re-sift the new root down
 *                                    through the shrunken heap
 *                                    arr[0..n − 1].
 *
 * Why TWO phases tracked separately
 *   Build does ≤ 2N comparisons (Floyd's bottom-up trick); extract is
 *   O(N log N).  Splitting them lets the renderer show the heap forming
 *   first, then the sorted suffix growing right-to-left — two very
 *   visually different "shapes" the eye can read.
 *
 * Members
 *   n           current heap size (number of elements still in the heap).
 *               Starts at N_ELEMS; decreases by 1 each extract.
 *   sift_from   index where the next heap_sift call starts.  In BUILD
 *               decreases from N/2 − 1 → 0; in EXTRACT always 0.
 *   phase       HEAP_PHASE_BUILD = building max-heap (Floyd O(N) heapify).
 *               HEAP_PHASE_EXTRACT = popping max + shrinking the heap.
 *
 * Invariants
 *   sift_from ≥ −1  AND  0 ≤ n ≤ N_ELEMS
 *   BUILD:   for every k in (sift_from..N − 1], the subtree rooted at k
 *            already satisfies the max-heap property.  n stays = N_ELEMS.
 *   EXTRACT: arr[0..n − 1] IS a max-heap; arr[n..N − 1] is sorted in
 *            non-decreasing order AND every value in it is ≥ everything
 *            in arr[0..n − 1].
 *
 * References
 *   [2] Williams 1964 — "Algorithm 232: Heapsort", CACM 7(6); the
 *       original publication of the heap data structure.
 *   [4] Knuth TAOCP Vol. 3 §5.2.3 — heapsort analysis (Floyd's O(N)
 *       build vs naive O(N log N) build).
 *   [5] Sedgewick & Wayne §2.4 — modern presentation; priority-queue
 *       framing of heapsort.
 */
typedef struct {
    int n;
    int sift_from;
    int phase;
} HeapState;

/*
 * AlgState — bundle of every algorithm's cursor state.
 *
 * Intent
 *   All five per-algorithm coroutine cursors live here side-by-side.
 *   Only the one matching Scene.alg is "active" at any moment; the
 *   step function dispatched via STEP_FN[alg] reads/writes its block
 *   exclusively.  init_alg() resets just the active block; the others
 *   retain whatever value they had (irrelevant since they are not read).
 *
 * Why STRUCT-of-structs (not a union)
 *   • Union would save ~640 bytes — negligible at N_ELEMS = 48.
 *   • Struct keeps each algorithm's state TYPED at access:
 *       sc->algs.bubble.i      vs.   sc->algs.u.bubble.i
 *     and reachable from a debugger without union-tag bookkeeping.
 *   • Tab-switching between algorithms (handle_input '\t') calls
 *     scramble() which re-inits just the new active block.
 *
 * Members
 *   bubble   BubbleState   — pass + pair cursors                  [§5.1]
 *   insert   InsertState   — outer + slide cursors                [§5.2]
 *   select   SelectState   — boundary + scan + argmin cursors     [§5.3]
 *   quick    QuickState    — explicit range stack + Lomuto cursors[§5.4]
 *   heap     HeapState     — phase + sift_from + heap size n      [§5.5]
 *
 * Invariants
 *   At any time, ONLY algs.<matching alg> is guaranteed consistent.
 *   Switching alg without calling init_alg() is undefined; the public
 *   path (TAB key) always re-scrambles + re-inits.
 *
 * References
 *   [7] Knuth TAOCP Vol. 1 §1.4.2 — coroutine state grouping.
 */
typedef struct {
    BubbleState bubble;
    InsertState insert;
    SelectState select;
    QuickState  quick;
    HeapState   heap;
} AlgState;

/*
 * Scene — owns ALL simulation + render state for one run.
 *
 * Intent
 *   One instance lives in main() and is passed by pointer to every
 *   sort step, draw function, and input handler.  This is the single
 *   "context object" for the program: pure functions take `const
 *   Scene *`, mutating functions take `Scene *`, and the signature
 *   alone communicates which kind of access each callee performs.
 *
 * Why a SCENE struct (not file-scope globals)
 *   • Replaceability: future variants (a side-by-side race of two
 *     algorithms, a recorded-trace replayer) can allocate multiple
 *     Scenes — impossible with globals.
 *   • Testability: a step function is now a pure transformation
 *     (state, hints) → (state', hints'); no hidden inputs.
 *   • Reading aid: every algorithm helper begins with `Scene *sc`,
 *     giving the reader one place to look for "what state exists".
 *
 * Members
 *   ── Data being sorted ───────────────────────────────────────────
 *   arr[N_ELEMS]      the int array under sort (values 1..N when fresh)
 *
 *   ── Algorithm dispatch ─────────────────────────────────────────
 *   alg               which algorithm runs this run (ALG_BUBBLE..ALG_HEAP)
 *   algs              all 5 per-algorithm cursor states (one active)
 *
 *   ── Per-tick render hints ──────────────────────────────────────
 *   hints             cmp1/cmp2 (gold) + swp1/swp2 (red) for THIS tick
 *
 *   ── Statistics + run flags ─────────────────────────────────────
 *   cmp_count         total compares since init_alg              (HUD)
 *   swp_count         total swaps   since init_alg              (HUD)
 *   done              true when the active step fn signalled completion
 *
 *   ── UI controls ────────────────────────────────────────────────
 *   paused            true → step_simulation is a no-op   (toggle: 'p')
 *   speed             steps per frame, ∈ [1, STEPS_MAX]   (adjust: +/-)
 *
 *   ── Terminal extent (refreshed each frame) ─────────────────────
 *   rows, cols        LINES / COLS snapshot for the current frame
 *
 * Invariants
 *   arr[] is always a permutation of 1..N_ELEMS (Fisher-Yates preserves it).
 *   0 ≤ alg < ALG_COUNT  (handle_input mod-cycles through the enum).
 *   1 ≤ speed ≤ STEPS_MAX  (handle_input clamps on every adjustment).
 *   `done == true` for the active algorithm implies arr[] is fully sorted.
 *   rows, cols match getmaxyx(stdscr, …) for the most recent frame.
 *
 * References
 *   [5] Sedgewick & Wayne §2.5 — comparing sort algorithms head-to-head;
 *       the visualiser is essentially a live version of those bench
 *       tables.
 *   [9] Bostock — "Visualizing Algorithms"; the single-state-object
 *       pattern for animated algorithm visualisers.
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

/* ===================================================================== */
/* §5  sort state machines                                                */
/* ===================================================================== */

/* ── shared tick utilities (used by every step function) ─────────────── */

/* Clear the "comparing now" hint slots — call when a tick has no compare. */
static void hints_clear_compare_pair(RenderHints *h) { h->cmp1 = h->cmp2 = -1; }

/* Clear the "just swapped" hint slots — call when a tick has no swap.    */
static void hints_clear_swap_pair(RenderHints *h)    { h->swp1 = h->swp2 = -1; }

/* Signal "this algorithm has finished".  Returns 1 so a tick can write
 * `return mark_sort_complete(sc);` — the universal "done" yield value.  */
static int mark_sort_complete(Scene *sc) { sc->done = true; return 1; }

/* Record one compare on the running total; renderer uses cmp_count in HUD. */
static void record_compare(Scene *sc, int a, int b)
{
    sc->hints.cmp1 = a; sc->hints.cmp2 = b;
    sc->cmp_count++;
}

/*
 * arr_swap — swap two elements of sc->arr and record the swap.
 *   Sets hints.swp1/swp2 for THIS tick so the renderer paints the
 *   two glyphs red-bold; bumps the run total.
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

/* Insertion sort begins after the trivial 1-element prefix; first
 * candidate to insert is arr[1], starting its slide from j=1. */
static void insert_state_seed(InsertState *ins)
{
    *ins = (InsertState){ .i = 1, .j = 1 };
}

/* Selection sort begins with the empty sorted prefix; the scan walks
 * j = 1..N − 1 looking for a min, with running argmin = i. */
static void select_state_seed(SelectState *sel)
{
    *sel = (SelectState){ .i = 0, .j = 1, .min = 0 };
}

/* Quicksort begins with one pending range covering the whole array,
 * Lomuto cursors primed for the first partition pass.  The classic
 * "i = lo − 1" Lomuto invariant means the first pre-increment lands
 * at i = lo on the first hit of a value ≤ pivot. */
static void quick_state_seed(QuickState *q)
{
    *q = (QuickState){
        .top    = 0,
        .cur_lo = 0,    .cur_hi = N_ELEMS - 1,
        .i      = -1,   .j      = 0,            /* Lomuto: i = lo − 1 */
        .phase  = QS_PHASE_SCAN,
    };
}

/* Heapsort begins in build phase, starting sift at the deepest
 * non-leaf (index N/2 − 1) and walking up to the root — Floyd's O(N)
 * bottom-up heapify. */
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
 * init_alg — reset shared run state, then seed the chosen algorithm's
 * cursors so its step function starts from the beginning.
 *
 *   Pseudocode:
 *     reset hints + counters + done flag
 *     dispatch on alg → seed its cursor state
 *
 *   Only the cursors of the SELECTED algorithm matter; the other four
 *   blocks retain stale values (irrelevant — they aren't read until
 *   init_alg runs for that algorithm).
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

/* ── bubble sort ─────────────────────────────────────────────────────── */

/* Have we finished every outer pass?  After N − 1 passes the entire
 * array is in final order (each pass parks one max element). */
static bool bubble_all_passes_done(const BubbleState *b)
{
    return b->i >= N_ELEMS - 1;
}

/* Compare the adjacent pair (j, j+1) and swap if out of order.
 * Records the compare hint and (if it happens) the swap. */
static void bubble_compare_adjacent_pair(Scene *sc, const BubbleState *b)
{
    record_compare(sc, b->j, b->j + 1);
    hints_clear_swap_pair(&sc->hints);
    if (sc->arr[b->j] > sc->arr[b->j + 1])
        arr_swap(sc, b->j, b->j + 1);
}

/* Advance j; if it just walked off the end of the unsorted prefix
 * (which shrinks by one each pass), wrap to j=0 and bump pass count. */
static void bubble_advance_cursor(BubbleState *b)
{
    if (++b->j >= N_ELEMS - 1 - b->i) { b->j = 0; b->i++; }
}

/*
 * bubble_step — one adjacent compare-and-maybe-swap.
 *
 *   Pseudocode:
 *     if all passes done:           mark complete
 *     compare adjacent pair (j, j+1); swap if out of order
 *     advance cursor (j++, wrap pass when prefix end reached)
 */
static int bubble_step(Scene *sc)
{
    BubbleState *b = &sc->algs.bubble;
    if (bubble_all_passes_done(b)) return mark_sort_complete(sc);

    bubble_compare_adjacent_pair(sc, b);
    bubble_advance_cursor(b);
    return 0;
}

/* ── insertion sort ──────────────────────────────────────────────────── */

/* Have we tried to insert every element?  After ins->i passes the
 * sorted prefix has consumed the whole array. */
static bool insert_all_done(const InsertState *ins)
{
    return ins->i >= N_ELEMS;
}

/* Compare arr[j-1] vs arr[j]; returns true iff the slide must continue
 * (left neighbour bigger, j still in bounds).  Always records the
 * compare hint and bumps cmp_count — this IS the per-tick comparison. */
static bool insert_left_neighbour_is_bigger(Scene *sc, const InsertState *ins)
{
    record_compare(sc, ins->j - 1, ins->j);
    hints_clear_swap_pair(&sc->hints);
    return ins->j > 0 && sc->arr[ins->j - 1] > sc->arr[ins->j];
}

/* Slide the current element one cell left: swap arr[j-1] with arr[j]
 * and decrement j.  Will be called repeatedly until the element finds
 * its slot. */
static void insert_slide_one_cell_left(Scene *sc, InsertState *ins)
{
    arr_swap(sc, ins->j - 1, ins->j);
    ins->j--;
}

/* The current element is in place; advance the outer cursor and start
 * sliding the NEXT candidate from its initial position. */
static void insert_pick_next_candidate(InsertState *ins)
{
    ins->i++;
    ins->j = ins->i;
}

/*
 * insert_step — slide the current element left until it lands in place.
 *
 *   Pseudocode:
 *     if all done:                       mark complete
 *     if left neighbour is bigger:       slide one cell left
 *     else:                              pick next candidate to insert
 *
 *   Each "swap then j--" is one slide tick; the outer i tracks which
 *   element we're currently placing.  Stable.
 */
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

/* ── selection sort ──────────────────────────────────────────────────── */

/* Have we filled every sorted slot but the last?  The trailing element
 * is automatically the maximum once N − 1 selections are done. */
static bool select_all_done(const SelectState *sel)
{
    return sel->i >= N_ELEMS - 1;
}

/* Is the inner scan still walking the unsorted suffix? */
static bool select_scan_in_progress(const SelectState *sel)
{
    return sel->j < N_ELEMS;
}

/* One scan tick: compare arr[j] against the running argmin; update
 * argmin if the new candidate is smaller; advance scan cursor. */
static void select_update_running_min(Scene *sc, SelectState *sel)
{
    record_compare(sc, sel->min, sel->j);
    if (sc->arr[sel->j] < sc->arr[sel->min]) sel->min = sel->j;
    sel->j++;
}

/* Scan complete: park the located minimum at slot i, advance the
 * sorted-prefix boundary, and prime the next scan starting at i+1. */
static void select_emit_min_and_restart_scan(Scene *sc, SelectState *sel)
{
    if (sel->min != sel->i) arr_swap(sc, sel->i, sel->min);
    sel->i++;
    sel->j   = sel->i + 1;
    sel->min = sel->i;
}

/*
 * select_step — one comparison toward the running min, or one emit-swap.
 *
 *   Pseudocode:
 *     if all done:                      mark complete
 *     if scan in progress:              update running min, advance j
 *     else (scan finished):             emit min into slot i, restart scan
 *
 *   One pass over the unsorted suffix to find the minimum, then ONE
 *   swap to put it in place.  Compare count is input-blind (always
 *   ~N²/2); swap count is at most N.
 */
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

/* ── quicksort: explicit range stack (replaces C recursion) ──────────── */

/* Push a pending sub-range [lo, hi] onto QuickState's explicit stack.
 * Trivial ranges (lo ≥ hi) are filtered at the call site; stack-overflow
 * guarding is per Sedgewick 1978 (capacity QS_STACK = 128). */
static void range_stack_push(QuickState *q, int lo, int hi)
{
    if (lo >= hi || q->top >= QS_STACK - 1) return;
    q->lo[q->top] = lo;
    q->hi[q->top] = hi;
    q->top++;
}

/* Pop ranges from the explicit stack, skipping any that have shrunk to
 * a single element.  On success writes (lo, hi) and returns true; on
 * empty stack returns false (the algorithm is finished). */
static bool range_stack_pop_next_valid(QuickState *q, int *out_lo, int *out_hi)
{
    while (q->top > 0) {
        q->top--;
        int lo = q->lo[q->top], hi = q->hi[q->top];
        if (lo < hi) { *out_lo = lo; *out_hi = hi; return true; }
    }
    return false;
}

/* ── quicksort: Lomuto partition mechanics ───────────────────────────── */

/* Prime the Lomuto cursors for a fresh sub-range [lo, hi]: i = lo − 1
 * (so first ++i lands on lo when a value ≤ pivot is found); j = lo. */
static void lomuto_prime_partition(QuickState *q, int lo, int hi)
{
    q->cur_lo = lo;  q->cur_hi = hi;
    q->i      = lo - 1;
    q->j      = lo;
    q->phase  = QS_PHASE_SCAN;
}

/* One tick of the Lomuto partition scan: compare arr[j] against the
 * pivot at arr[cur_hi].  If smaller-or-equal, grow the "≤ pivot"
 * prefix by swapping arr[++i] ↔ arr[j].  Always advance j. */
static void lomuto_partition_tick(Scene *sc, QuickState *q)
{
    record_compare(sc, q->j, q->cur_hi);
    if (sc->arr[q->j] <= sc->arr[q->cur_hi]) {
        q->i++;
        arr_swap(sc, q->i, q->j);
    }
    q->j++;
}

/* Scan finished for the current range: park the pivot at its final
 * position (i + 1), then push the two non-trivial sub-ranges so the
 * pop phase can resume on them. */
static void lomuto_finalize_pivot(Scene *sc, QuickState *q)
{
    q->i++;
    if (q->i != q->cur_hi) arr_swap(sc, q->i, q->cur_hi);
    range_stack_push(q, q->cur_lo, q->i - 1);    /* left  sub-range */
    range_stack_push(q, q->i + 1,  q->cur_hi);   /* right sub-range */
    q->phase = QS_PHASE_POP;
}

/* Is the partition scan still walking the current range? */
static bool quick_partition_in_progress(const QuickState *q)
{
    return q->j < q->cur_hi;
}

/*
 * quick_step — one tick of iterative Lomuto quicksort.
 *
 *   Pseudocode:
 *     clear hint pairs
 *     if SCAN phase:
 *       if partition in progress:   one Lomuto compare-and-maybe-swap
 *       else:                       finalize pivot + push sub-ranges
 *       return 0
 *     ── POP phase ──
 *     if a valid sub-range pops:    prime new partition, return 0
 *     else:                         mark complete
 *
 *   The explicit range stack replaces C recursion (Sedgewick 1978);
 *   yielding once per Lomuto compare makes every swap visible.
 */
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
 * heap_sift — restore the max-heap property at `root`.
 *
 *   Walks the LARGER child downward until either the heap rule holds
 *   (parent ≥ both children) or we reach a leaf.  Multiple swaps per
 *   call — the only multi-swap helper in §5.  The other four step
 *   functions yield after every compare-or-swap; heap_sift batches
 *   the cascade into one tick because splitting it midway would
 *   leave the heap in a transient invalid state visible to the renderer.
 *
 *   `end` is the inclusive last index of the LIVE heap (excludes the
 *   already-extracted sorted suffix).
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

/* ── heapsort: phase predicates ──────────────────────────────────────── */

/* Floyd's bottom-up build phase is still working its way up to the root. */
static bool heap_build_in_progress(const HeapState *h)
{
    return h->sift_from >= 0;
}

/* Extract phase has shrunk the live heap to nothing — everything sorted. */
static bool heap_extract_done(const HeapState *h)
{
    return h->n <= 0;
}

/* ── heapsort: per-tick operations ───────────────────────────────────── */

/* Build-phase tick: sift down from the current starting point, then
 * step the starting point one cell closer to the root (Floyd O(N)
 * heapify; cf. Williams 1964 + Floyd 1964). */
static void heap_build_sift_next(Scene *sc, HeapState *h)
{
    heap_sift(sc, h->sift_from, N_ELEMS - 1);
    h->sift_from--;
}

/* Build phase finished — transition to extract phase.  The heap now
 * covers all of arr[]; the first extract will park the max at the end. */
static void heap_transition_to_extract(HeapState *h)
{
    h->phase = HEAP_PHASE_EXTRACT;
    h->n     = N_ELEMS - 1;
}

/* Extract-phase tick: swap the heap root (current max) into the last
 * live slot, shrink the live heap by one, then sift the new root down
 * through the shortened heap. */
static void heap_extract_one_max(Scene *sc, HeapState *h)
{
    arr_swap(sc, 0, h->n);
    h->n--;
    heap_sift(sc, 0, h->n);
}

/*
 * heap_step — one sift-down (build phase) or one extract (extract phase).
 *
 *   Pseudocode:
 *     clear hint pairs
 *     if BUILD phase:
 *       if build in progress:        sift one subtree, step toward root
 *       else:                        transition to EXTRACT
 *     if extract done:               mark complete
 *     extract one max element
 *
 *   Build establishes the max-heap (Floyd's O(N) bottom-up).  Extract
 *   shrinks the live heap by one each tick, parking the extracted max
 *   at the end — building the sorted suffix right-to-left.
 */
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

/* Step-function dispatch table — one entry per enum Alg value. */
typedef int (*StepFn)(Scene *);
static const StepFn STEP_FN[ALG_COUNT] = {
    bubble_step, insert_step, select_step, quick_step, heap_step
};

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/* Fill arr with the identity permutation 1..N — the canonical sorted
 * baseline that the shuffler then disturbs. */
static void arr_fill_identity(Scene *sc)
{
    for (int i = 0; i < N_ELEMS; i++) sc->arr[i] = i + 1;
}

/* Fisher-Yates in-place uniform shuffle (Knuth TAOCP Vol. 2 §3.4.2,
 * Algorithm P).  Walking i from N − 1 down to 1, swap arr[i] with a
 * uniformly random arr[j] where 0 ≤ j ≤ i.  Each of the N! permutations
 * is produced with equal probability. */
static void arr_fisher_yates_shuffle(Scene *sc)
{
    for (int i = N_ELEMS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = sc->arr[i]; sc->arr[i] = sc->arr[j]; sc->arr[j] = t;
    }
}

/*
 * scramble — refill arr with 1..N, shuffle uniformly, then reset the
 * active algorithm's cursors so the new run starts cleanly.
 *
 *   Pseudocode:
 *     arr_fill_identity(sc)              ← arr := [1, 2, …, N]
 *     arr_fisher_yates_shuffle(sc)       ← uniform random permutation
 *     init_alg(sc, sc->alg)              ← reset cursors + counters
 */
static void scramble(Scene *sc)
{
    arr_fill_identity(sc);
    arr_fisher_yates_shuffle(sc);
    init_alg(sc, sc->alg);
}

/*
 * mark_cell — bounds-checked mvaddch with the canonical sign-safe cast.
 *
 *   The (chtype)(unsigned char) double-cast prevents sign-extension
 *   corruption on glyphs above 0x7F — ncurses' mvaddch interprets a
 *   negative chtype as a control byte.  Routing all bar drawing
 *   through this single helper gives us ONE clip + cast site instead
 *   of N inlined repetitions.
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
 * bar_pair — colour-pair classifier for bar index `i`.
 *
 *   Priority order (top wins):
 *     just-swapped  → PAIR_SWP   (red bold)
 *     comparing now → PAIR_CMP   (gold)
 *     fully sorted  → PAIR_SORT  (green bold)
 *     otherwise     → PAIR_NORM  (grey)
 *
 *   Swap beats compare so a freshly-swapped pair flashes red instead
 *   of gold for one frame, making the swap visible even when the
 *   algorithm immediately moves on to a new compare.
 */
static int bar_pair(const Scene *sc, int i)
{
    if (i == sc->hints.swp1 || i == sc->hints.swp2) return PAIR_SWP;
    if (i == sc->hints.cmp1 || i == sc->hints.cmp2) return PAIR_CMP;
    if (sc->done)                                   return PAIR_SORT;
    return PAIR_NORM;
}

/* ── bar geometry: scaling value → screen cells ──────────────────────── */

/* Linear scale: an arr[i] of size `value` (1..N) maps to a bar of
 * height `value · bar_max / N` cells.  bar_max is the tallest bar a
 * full-height value gets — passed in once, computed by draw_bars. */
static int bar_value_to_cell_height(int value, int bar_max)
{
    return value * bar_max / N_ELEMS;
}

/* Compute the half-open column span [col_s, col_e) covered by bar i.
 * Bars are spaced uniformly across the available cols; the last bar
 * is clamped so we never write past the right edge. */
static void bar_column_span(int i, int cols, int *out_s, int *out_e)
{
    int s = i       * cols / N_ELEMS;
    int e = (i + 1) * cols / N_ELEMS;
    if (e > cols) e = cols;
    *out_s = s;  *out_e = e;
}

/* PAIR_SWP and PAIR_SORT use A_BOLD for the "look at me" flash; the
 * other two pairs (compare and norm) render in their plain weight. */
static int bar_attr_for_pair(int pair)
{
    return (pair == PAIR_SWP || pair == PAIR_SORT) ? A_BOLD : 0;
}

/* Paint a single bar column-span: fill '#' from the bottom row upward
 * for bar_h cells, stopping when we hit the HUD reservation or run out
 * of bar height.  Bar grows from the bottom — the visual "ground". */
static void paint_bar_column(const Scene *sc, int col_s, int col_e,
                             int bar_h, int pair, int attr)
{
    for (int row = sc->rows - 1; row >= HUD_ROWS; row--) {
        int depth_from_bottom = sc->rows - 1 - row;
        if (depth_from_bottom >= bar_h) break;   /* above the bar — done */
        for (int c = col_s; c < col_e; c++)
            mark_cell(sc, row, c, '#', pair, attr);
    }
}

/*
 * draw_bar — paint one vertical bar of '#' characters from the bottom up.
 *
 *   Pseudocode:
 *     bar_h      := bar_value_to_cell_height(arr[i], bar_max)
 *     [s, e)     := bar_column_span(i, cols)
 *     pair, attr := classify this bar's render state
 *     paint_bar_column from the bottom up for bar_h cells
 */
static void draw_bar(const Scene *sc, int i, int bar_max)
{
    int bar_h = bar_value_to_cell_height(sc->arr[i], bar_max);
    int col_s, col_e;
    bar_column_span(i, sc->cols, &col_s, &col_e);

    int pair = bar_pair(sc, i);
    int attr = bar_attr_for_pair(pair);
    paint_bar_column(sc, col_s, col_e, bar_h, pair, attr);
}

/* Compute the tallest bar a full-height value gets, leaving HUD_ROWS
 * reserved at the top.  Guard against degenerate terminals where the
 * bar area is zero or one cell tall. */
static int bar_area_max_height(const Scene *sc)
{
    int bar_area = sc->rows - HUD_ROWS;
    return bar_area > 1 ? bar_area - 1 : 1;
}

/* draw_bars — iterate every bar; compute the scaling once. */
static void draw_bars(const Scene *sc)
{
    int bar_max = bar_area_max_height(sc);
    for (int i = 0; i < N_ELEMS; i++) draw_bar(sc, i, bar_max);
}

/* ── HUD formatting + drawing ────────────────────────────────────────── */

/* One-word run-state indicator used in the top status string. */
static const char *run_state_label(const Scene *sc)
{
    if (sc->done)   return "SORTED";
    if (sc->paused) return "PAUSED";
    return "running";
}

/* Format the top status line into buf — "Bubble speed:4x N=48 cmp:123
 * swp:11 running" etc.  Buffer must hold ≥ 80 chars to fit any N. */
static void format_hud_status(const Scene *sc, char *buf, size_t bufsz)
{
    snprintf(buf, bufsz,
             " %s  speed:%dx  N=%d  cmp:%lld  swp:%lld  %s ",
             ALG_NAME[sc->alg], sc->speed, N_ELEMS,
             sc->cmp_count, sc->swp_count, run_state_label(sc));
}

/* Paint the right-aligned status line at row 0 in CP_HUD + A_BOLD. */
static void draw_top_status(const Scene *sc, const char *status)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    int pad = sc->cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Paint the bottom hint strip listing every interactive key. */
static void draw_bottom_hint(const Scene *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
        " q:quit  TAB:next-alg  spc:scramble  p:pause  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * draw_hud — top status line (right-aligned) + bottom action hints.
 *
 *   Pseudocode:
 *     format the status line into a local buffer
 *     draw it right-aligned on row 0       (CP_HUD,  A_BOLD)
 *     draw the key-hint strip on last row  (CP_HINT, A_BOLD)
 *
 *   Layout follows the project HUD Standard (CLAUDE.md): data on top,
 *   actions on bottom, both bright and bold so they remain legible
 *   against any animation cell underneath.
 */
static void draw_hud(const Scene *sc)
{
    char status[80];
    format_hud_status(sc, status, sizeof status);
    draw_top_status(sc, status);
    draw_bottom_hint(sc);
}

/*
 * scene_draw — render one frame: bars then HUD.
 *
 *   Painter's order: bars first (background), HUD last so its glyphs
 *   sit on top of any bar that intrudes into the top status row.
 */
static void scene_draw(const Scene *sc)
{
    draw_bars(sc);
    draw_hud(sc);
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

/* True when step_simulation should do nothing this frame. */
static bool simulation_idle(const Scene *sc)
{
    return sc->paused || sc->done;
}

/*
 * step_simulation — advance the active algorithm by `speed` micro-steps.
 *
 *   Pseudocode:
 *     if idle (paused or already sorted):  return
 *     repeat `speed` times:
 *       tick the active algorithm's step fn
 *       if it signalled "done": break
 *
 *   `speed` is the visualisation tempo: STEPS_DEFAULT (1) shows every
 *   operation; higher values batch multiple operations into one frame
 *   so quick / heap don't crawl.  Early-exiting the moment a step
 *   reports done avoids burning the rest of the budget on a sorted array.
 */
static void step_simulation(Scene *sc)
{
    if (simulation_idle(sc)) return;
    StepFn tick = STEP_FN[sc->alg];
    for (int s = 0; s < sc->speed; s++)
        if (tick(sc)) break;
}

/* ── one-shot init helpers for main() ────────────────────────────────── */

/* Install the POSIX signal handlers + atexit cleanup.  SIGINT/SIGTERM
 * request graceful shutdown; SIGWINCH triggers a deferred resize. */
static void signals_install(void)
{
    atexit(cleanup);
    signal(SIGINT,  sig_h);
    signal(SIGTERM, sig_h);
    signal(SIGWINCH, sig_h);
}

/* Configure ncurses for the standard "raw input, no echo, no cursor,
 * non-blocking getch" interactive mode shared by every demo.  See
 * "Common ncurses Bugs" in CLAUDE.md for the typeahead(-1) rationale. */
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

/* Snapshot the initial Scene: default algorithm + speed, current
 * terminal extent, and a freshly scrambled array.  Called once at
 * startup, after terminal_init has given us a valid stdscr. */
static void scene_init(Scene *sc)
{
    sc->alg   = ALG_BUBBLE;
    sc->speed = STEPS_DEFAULT;
    getmaxyx(stdscr, sc->rows, sc->cols);
    scramble(sc);
}

/* If the SIGWINCH handler set g_resize, tear ncurses down + back up so
 * stdscr matches the new terminal extent, then refresh sc->rows/cols. */
static void handle_resize_if_pending(Scene *sc)
{
    if (!g_resize) return;
    g_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Drain at most one keypress per frame and route it through the
 * Scene-aware input handler.  ncurses is in non-blocking mode so
 * getch() returns ERR when no key is pending. */
static void drain_input(Scene *sc)
{
    int ch = getch();
    if (ch != ERR) handle_input(sc, ch);
}

/* Render one frame: erase the back buffer, draw the scene, mark
 * stdscr dirty, then issue ONE diffed write via doupdate(). */
static void frame_render(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Sleep just long enough to keep the frame on its TARGET_FPS budget.
 * `frame_start_ns` is the clock reading taken at the top of this frame;
 * elapsed = now − start.  If we ran over budget, clock_sleep_ns is a
 * no-op (it short-circuits on non-positive durations). */
static void wait_until_next_frame(long long frame_start_ns)
{
    long long elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(FRAME_NS - elapsed);
}

/*
 * main — wire up ncurses + signals, allocate the Scene on the stack,
 * then run the canonical loop:
 *
 *   Pseudocode:
 *     install signal handlers + atexit cleanup
 *     bring up terminal (ncurses + colours)
 *     seed Scene (defaults + extent + scrambled array)
 *
 *     while not quit:
 *       handle resize if pending
 *       drain one keypress  → handle_input
 *       advance simulation  → step_simulation
 *       render frame
 *       wait until next frame boundary
 *
 *   Scene lives on main's stack — no malloc anywhere in the program.
 *   g_quit / g_resize remain file-scope because POSIX signal handlers
 *   can only safely touch volatile sig_atomic_t globals.
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
