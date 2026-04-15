# Concept: Sorting Algorithm Visualizer

## Pass 1 — Understanding

### Core Idea
Visualize classic comparison-based sorting algorithms in real time. Each array element is a bar; comparisons and swaps are animated step-by-step. Algorithms: bubble sort, insertion sort, selection sort, quicksort, heapsort.

### Mental Model
An array of bars of different heights. Watch them move around as the algorithm runs. Bubble sort slowly moves large bars to the right. Quicksort recursively partitions around a pivot. Heapsort builds a heap then extracts in order. The visualization reveals why some algorithms are faster than others.

### Key Complexity Facts
| Algorithm | Best | Average | Worst | Stable |
|-----------|------|---------|-------|--------|
| Bubble | O(n) | O(n²) | O(n²) | Yes |
| Insertion | O(n) | O(n²) | O(n²) | Yes |
| Selection | O(n²) | O(n²) | O(n²) | No |
| Quick | O(n log n) | O(n log n) | O(n²) | No |
| Heap | O(n log n) | O(n log n) | O(n log n) | No |

### Data Structures
- `arr[N]`: int array, values 1..N
- `highlight[N]`: which indices are currently being compared/swapped
- State machine: sorting proceeds one step per frame

### Non-Obvious Decisions
- **Generator/coroutine pattern**: Sorting algorithms are inherently sequential, but visualization needs to pause after each step. Implement as a state machine or store a queue of operations.
- **Step queue**: Before visualizing, run the sort normally but record every comparison and swap. Then replay the queue one operation per frame. Much simpler than making the sort algorithm itself pause.
- **Color coding**: Compared elements (red), swapped elements (green/yellow), sorted region (blue), pivot (white).
- **Variable speed**: Let user press +/- to change how many steps per frame.
- **Statistics**: Count total comparisons and swaps per run. Display side-by-side with the bar chart.

## From the Source

**Algorithm:** Five sorting algorithms animated one-operation-per-tick: Bubble sort (O(n²), stable, adjacent swaps), Insertion sort (O(n²) shifts, stable, builds sorted prefix), Selection sort (O(n²) comparisons, O(n) swaps, not stable), Quicksort (O(n log n) average, Lomuto partition: pivot = last element), Heapsort (O(n log n) worst-case, in-place, not stable — max-heap built in O(n), then extracted).

**Data-structure:** Coroutine-style iterators: each algorithm is implemented as a state machine (struct + step function) that advances exactly one compare-or-swap per call. This allows the animation loop to run at a user-controlled rate without threads.

**Rendering:** Vertical bar chart: element value → bar height in `#` characters. Colour encodes operation state: grey=unsorted, yellow=currently comparing, red=just swapped, green=sorted (in final position). N_ELEMS=48 bars fill a typical 80-column terminal.

---

### Key Constants
| Name | Role |
|------|------|
| N | array size (terminal width - margins) |
| STEPS_PER_FRAME | animation speed |
| HIGHLIGHT_FRAMES | how long to show a comparison/swap |

### Open Questions
- Does quicksort's worst case (already-sorted input) show visually?
- Can you run two algorithms simultaneously (side by side) for comparison?
- Implement merge sort — how does the visualization differ?

---

## Pass 2 — Implementation

### Pseudocode
```
# Record-then-replay approach
Operation = {type: CMP|SWAP, i, j}

record_bubblesort(arr) → ops[]:
    for i in n-1..0:
        for j in 0..i:
            ops.push(CMP, j, j+1)
            if arr[j] > arr[j+1]:
                swap(arr[j], arr[j+1])
                ops.push(SWAP, j, j+1)

# Replay
state = {ops, pos=0, arr=original_arr}

step():
    op = ops[state.pos++]
    if op.type == SWAP: swap(arr[op.i], arr[op.j])
    highlight = (op.i, op.j)

draw():
    for i in 0..N:
        bar_height = arr[i] * (rows-HUD) / N
        color = HIGHLIGHT if i in highlight else BAR
        for row in rows-bar_height..rows:
            mvaddch(row, i*bar_width, '█')
    draw_stats(comparisons, swaps, algorithm_name)
```

### Module Map
```
§1 config    — N, STEPS_PER_FRAME, algorithm list
§2 record    — record_bubble(), record_quick(), etc.
§3 replay    — step(), state machine
§4 draw      — bar chart with highlight + stats HUD
§5 app       — main loop, keys (algo select, speed, reset, shuffle)
```

### Data Flow
```
random array → record sort ops → operation queue
→ replay one op per step → update arr + highlight
→ draw bar chart → screen
```
