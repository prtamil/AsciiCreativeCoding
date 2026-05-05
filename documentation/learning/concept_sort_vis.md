# Pass 1 — sort_vis.c: Coroutine-Style Sort Visualiser

## Core Idea

Every comparison sort is a sequence of two-element decisions: look at
a pair, decide if they're out of order, maybe swap. The five
algorithms here differ ONLY in the rule for choosing the next pair.
By rewriting each algorithm as an iterative state machine that emits
exactly one compare-or-swap per `step()` call, the visualiser can
pause, slow down, or fast-forward an algorithm WITHOUT threads,
setjmp coroutines, or recording-then-replay.

## Mental Model

Imagine 48 people of different heights waiting to line up shortest-to-
tallest. A coach barks out one rule and the line follows it:

| Algorithm  | Coach's rule                                                     |
|------------|------------------------------------------------------------------|
| Bubble     | "Compare your right neighbour; if shorter, trade."               |
| Insertion  | "You're new; walk left until the person beside you is shorter."  |
| Selection  | "Find the shortest unsorted person; bring them to the front."    |
| Quicksort  | "Pivot is whoever's at the right end; everyone shorter goes left." |
| Heapsort   | "Build a tournament bracket where parent beats child, peel champions." |

Each rule produces a different visual personality — bubble crawls,
insertion smooths the left, selection drags a lone marker, quicksort
splits, heap rebuilds. Watching them side by side teaches algorithm
intuition no textbook page can.

## State Machines, Not Coroutines

Naïvely, sorting is sequential: the algorithm runs to completion in one
pass. Visualisation requires pausing after each comparison or swap.
Three approaches exist:

1. **Threads.** Run the sort on its own thread, block on a semaphore
   between operations. Possible but heavy.
2. **Record-then-replay.** Run the sort once collecting an `ops[]` log,
   then animate by stepping through the log. Simple but doubles memory
   and decouples animation from the live algorithm.
3. **State-machine step()** ← what this file does. Each algorithm is
   a struct of cursor variables + a `step_fn()` that advances exactly
   one operation. The animation loop calls `step_fn()` at a controlled
   rate.

Approach 3 wins because:
- Each `step_fn()` returns 0 (more work) or 1 (sorted). The outer loop
  caps speed by counting return values.
- No memory overhead beyond cursor variables.
- The user can pause, rewind (via scramble + reset), or change speed
  mid-run without restarting.
- Each algorithm's full state is just a handful of int globals — easy
  to restart with `init_alg(alg)`.

## Per-Algorithm State

```c
/* Bubble    */ static int g_bi, g_bj;
/* Insertion */ static int g_ii, g_ij;
/* Selection */ static int g_si, g_sj, g_smin;
/* Quicksort */ static int g_qlo[QS_STACK], g_qhi[QS_STACK], g_qtop;
                static int g_ql, g_qh, g_qi, g_qj, g_qphase;
/* Heapsort  */ static int g_hn, g_hi_h, g_hphase;
```

Quicksort uses an explicit stack — the iterative version pushes
sub-ranges (lo, hi) instead of recursing. QS_STACK = 128 is enough
for any random N=48 array; adversarial inputs could push more, but
the visualiser silently caps it (a real implementation would split
the larger half manually).

## Per-Algorithm step()

| Algorithm | One step does                                                              |
|-----------|----------------------------------------------------------------------------|
| Bubble    | Compare arr[j] vs arr[j+1]; swap if needed; advance j; reset+inc i at row end |
| Insertion | Compare arr[j-1] vs arr[j]; swap+decrement j on inversion; else jump to next i |
| Selection | Walk j across unsorted suffix tracking g_smin; on j=N, swap to front       |
| Quicksort | Phase 0: scan one j vs pivot. Phase 1: place pivot, push sub-ranges, pop next |
| Heapsort  | Phase 0: sift-down one root. Phase 1: extract root, sift-down 0            |

**Atomicity exception: heap_sift().** Unlike the other four, heap's
sift-down walks all the way down in one call (potentially several
swaps). Per-swap heapsort would double the state needed (running
swap-cursor + parent-pointer) and the swap stream still looks visually
distinct, so this was an intentional simplification. EDGE CASES below.

## Operation Highlight Pattern

After every step, four globals carry the indices to highlight:
```c
static int g_cmp1 = -1, g_cmp2 = -1;   /* indices being compared */
static int g_swp1 = -1, g_swp2 = -1;   /* indices just swapped   */
```

The draw routine paints those bars in a contrasting colour:
- Compare → gold (220)
- Swap → bright red 196 + A_BOLD
- Sorted (when g_done) → matrix green 46 + A_BOLD
- Otherwise → light grey 250

Setting both pairs to `-1` at the start of every step prevents stale
highlights from one operation bleeding into the next. `arr_swap()`
clears `g_cmp*` implicitly; pure-compare paths clear `g_swp*`.

## Bar Rendering

Element value → bar height: `bar_h = arr[i] · bar_max / N_ELEMS`,
where `bar_max = rows - HUD_ROWS - 1`. Each bar gets a column band
`[i·cols/N, (i+1)·cols/N)` wide so wider terminals give thicker bars.

The drawing loop scans top-down and breaks as soon as `depth >= bar_h`
— above the bar, all cells are empty (we already `erase()`'d), so
there's no need to overwrite with spaces.

```c
for (int row = g_rows - 1; row >= HUD_ROWS; row--) {
    int depth = g_rows - 1 - row;
    if (depth >= bar_h) break;
    for (int c = col_s; c < col_e; c++)
        mark_cell(row, c, '#', pair, attr);
}
```

## Color Encoding

All pairs use bg = -1. Foregrounds in the bright half:

| Pair       | Color      | Used for                       |
|------------|------------|--------------------------------|
| PAIR_NORM  | 250 grey   | untouched bars                 |
| PAIR_CMP   | 220 gold   | compare pair (gold not yellow  |
|            |            | so it doesn't blur with HUD)   |
| PAIR_SWP   | 196 red    | swap pair (A_BOLD)             |
| PAIR_SORT  |  46 green  | finished pass (A_BOLD)         |
| PAIR_HUD   | 226 yellow | top-right status (A_BOLD)      |
| PAIR_HINT  |  51 cyan   | bottom key strip (A_BOLD)      |

Picking 220 (gold) for compare instead of 226 (yellow, used by HUD)
prevents the bar highlight from camouflaging into the status line
when both share the same column band.

## Worked Example (defaults: N=48, 30 fps, speed 1×)

Bubble at speed 1×:
- Worst case: 48 × 47 / 2 = 1128 compares + ≤1128 swaps = up to 2256
  ops. At 30 fps × 1 op/frame, that's 75 seconds — slow enough to
  watch every adjacent inversion fix itself.

Heapsort at speed 1×:
- Build phase: ~24 sift-down calls, each touching log₂(48) ≈ 6 levels.
  About 144 compare/swap ops total = 5 seconds.
- Extract phase: 47 swaps + 47 sift-downs ≈ 380 ops = 13 seconds.
- Total ~18 s vs bubble's 75 s — the algorithmic gap is visible.

Press `+` to double speed: 256× makes the entire heapsort finish in
one frame, which is fine — `step_simulation()` calls step_fn up to
g_speed times per frame and `if (step()) break` short-circuits as
soon as `g_done` is set.

## Counting Operations

```c
static long long g_cmp_count, g_swp_count;
```

Both increment on every comparison and swap respectively. The HUD
shows them live, and they reset to 0 in `init_alg()`. Useful for
verifying complexity: bubble & insertion are ~N²/2 compares;
heapsort & quicksort ~N·log₂(N) on random input.

## Lomuto Partition Invariant

After scanning some j in [lo+1, hi-1] with cursor i:
```
  arr[lo  .. i  ]   ≤ pivot
  arr[i+1 .. j-1]   > pivot
  arr[j   .. hi-1]  unscanned
  arr[hi]            = pivot
```

When j hits hi, swap arr[i+1] with arr[hi] to plant the pivot in its
final position; everything left is ≤ pivot, everything right is >.
Push left sub-range [lo, i] and right sub-range [i+2, hi] onto the
stack. The visualiser shows a snake of red swaps walking left-to-right
during the scan, then a single dramatic placement of the pivot.

## Module Map

```
§1 config  — N_ELEMS, HUD_ROWS, frame timing, color pair IDs
§2 clock   — clock_ns + clock_sleep_ns
§3 color   — bg=-1 + bright-half fg, gold instead of yellow for cmp
§5 sort    — five state-machine sorters + arr_swap + init_alg
§7 scene   — mark_cell + bar_pair + draw_bar + draw_bars + draw_hud
§8 app     — signals, resize, handle_key, step_simulation, main loop
```

## Edge Cases

- **Heap atomicity.** `heap_sift()` performs multiple swaps per call,
  so heapsort's animation runs faster per "step" than the other four.
  This is deliberate; one-swap-per-call would double the state needed
  and the swap stream still looks distinct enough.

- **Quicksort stack overflow.** Adversarial input (already sorted)
  pushes O(N) ranges into a 128-deep stack. With N=48 we're fine; at
  N=256+ we'd silently lose ranges. A production impl would push the
  larger half first and recurse on the smaller — visualiser glosses
  over this.

- **Speed loop short-circuit.** `step_simulation()` runs up to g_speed
  steps per frame but breaks as soon as a step returns 1 (done). Without
  the break, more steps would clobber `g_done` and re-execute from the
  cursor on the next frame.

- **Resize.** SIGWINCH refreshes `g_rows/g_cols` but keeps the array's
  current sort progress. The bar widths recompute on the next draw, so
  a mid-sort resize is visually harmless.

- **Speed = 0.** Halving past 1 saturates at 1; no "no progress per
  frame" mode.

## How to Verify

- Final state: `arr[i] == i + 1` for every i (we scrambled 1..N). All
  bars rise monotonically and turn green.

- Operation counts at N=48:
  - Bubble: ≤ 1128 compares + ≤ 1128 swaps
  - Insertion: ≤ 1128 compares + ≤ 1128 swaps (same worst case as bubble
    but average fewer because of the early-break per row)
  - Selection: exactly N(N-1)/2 = 1128 compares regardless of input,
    + at most N-1 = 47 swaps
  - Quicksort random: ~N·log₂(N) ≈ 268 compares
  - Heapsort: ~N·log₂(N) ≈ 268 compares + ≤ 2N·log₂(N) ≈ 535 swaps

- Doubling N from 48 to 96 quadruples bubble/select op counts and
  merely doubles heap/quick.

- Press TAB on a near-sorted array: bubble finishes in one pass with
  no swaps; selection still does N²/2 compares (input-blind).

## Open Questions

1. Implement merge sort. The state machine needs an auxiliary array —
   how does that change the animation visually (a "tape" sliding left
   to right)?
2. Run two algorithms simultaneously in split-screen, sharing the same
   scrambled array. Each tick advance both step machines; compare
   side-by-side how each evolves.
3. Make the worst-case quicksort visible: feed in already-sorted
   input. Watch O(N²) compares unfold and contrast with random input
   (~N·log N).
4. Add a "cocktail shaker" bidirectional bubble. Does alternating
   directions reduce visible "turtles" (small values stuck at the
   right end)?

## References

- Sedgewick & Wayne, *Algorithms* 4e, chs. 2.1–2.4 (bubble, insertion,
  selection, quick).
- Cormen et al., *Introduction to Algorithms* 3e, ch. 6 (heapsort).
- Wikipedia, *Sorting algorithm*,
  https://en.wikipedia.org/wiki/Sorting_algorithm
- Mike Bostock, *Visualizing Algorithms*,
  https://bost.ocks.org/mike/algorithms/
