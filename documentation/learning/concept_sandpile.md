# Pass 1 — fractal_random/sandpile.c: Abelian Sandpile (BTW Model)

## Core Idea

Grains are dropped one at a time onto the centre of a grid. Any cell that accumulates 4 or more grains topples — it gives 1 grain to each of its 4 cardinal neighbours and loses 4. This triggers a cascade (avalanche) that spreads until every cell holds 0–3 grains. The emergent long-run pattern on the grid has intricate fractal symmetry that was never designed in — it arises purely from the toppling rule.

## The Mental Model

Imagine filling a circular tray with sand, one grain at a time, always at the centre. Most grains just pile up. Occasionally one grain crosses a threshold and a small collapse ripples outward. Rarely, a single grain tips a cascade that crosses the entire tray. The system "self-organises" to the edge of stability — this is self-organised criticality (SOC).

The mathematical surprise is the long-run stable pattern: after millions of grains the pile develops a fractal/quasi-crystalline design — triangles inside triangles, spirals of colour — with no randomness and no external forcing.

## Avalanche Algorithm: BFS Queue

The naive approach (scan full grid for unstable cells, repeat) is O(rows × cols) per topple. The BFS queue is O(avalanche_size):

```c
static int     g_qr[QMAX], g_qc[QMAX];   /* row/col of queued cells */
static uint8_t g_inq[MAX_ROWS][MAX_COLS]; /* deduplication flag */
static int     g_qhead, g_qtail;

static void enq(int r, int c) {
    if (g_inq[r][c]) return;   /* already in queue — skip */
    g_inq[r][c] = 1;
    g_qr[g_qtail] = r;
    g_qc[g_qtail] = c;
    g_qtail = (g_qtail + 1) % QMAX;
}
```

`g_inq[]` prevents the same cell from entering the queue multiple times. Without it, a cell receiving grains from multiple neighbours could be enqueued 4 times, causing it to topple 4 times in one pass.

## The wave_end Bug (and Fix)

The original code used a `wave_end` snapshot:
```c
int wave_end = g_qtail;
while (g_qhead != wave_end) { ... }
```

This only processed cells that were in the queue at the start of `avalanche_step`. Cells enqueued *during* toppling (the newly unstable neighbours) were left pending. The cascade died after one ring — the pile never grew beyond one step from the centre.

The fix: loop until the queue is completely empty:
```c
while (g_qhead != g_qtail) { ... }
```

## Two Avalanche Modes

```c
if (!full) break;   /* vis mode: one topple per call */
```

- **`full=1` (instant mode):** drain the entire queue in one call. Drop many grains per frame; watch the fractal pattern grow.
- **`full=0` (vis mode):** process exactly one cell then return. One topple per video frame. Press `v` to toggle — you see each avalanche wave propagate cell by cell in red.

## Coloring by Grain Count

```c
static const char GRAIN_CH[4] = { ' ', '.', '+', '#' };
/* 0=blank, 1=dim-blue '.', 2=green '+', 3=bright-gold '#' */
```

Cells with grain count ≥ 4 (temporarily unstable, only visible in vis mode) are drawn as bright-red `*`. The drop point is marked `@` when empty.

## Non-Obvious Decisions

### Why is the queue circular, not a straight array?
After a large avalanche, `g_qtail` wraps around. A straight array would need to be of size `MAX_ROWS × MAX_COLS` and could overflow if the avalanche enqueues the same cells repeatedly. The circular queue with `g_inq` deduplication guarantees each cell is in the queue at most once — so `QMAX = MAX_ROWS × MAX_COLS + 1` is sufficient.

### Why does `drop_grain` reset the queue?
```c
if (g_grid[g_cr][g_cc] >= 4) {
    memset(g_inq, 0, sizeof(g_inq));
    g_qhead = g_qtail = 0;
    enq(g_cr, g_cc);
}
```
After `avalanche_step(1)` the queue is always empty (it drained to completion). But `drop_grain` resets defensively for the initial state and to clear any stale `g_inq` flags. The `memset` is O(MAX_ROWS × MAX_COLS) — but it only runs when the centre cell hits 4 (roughly every 4 drops), not every tick.

### Why does the fractal pattern have 4-fold symmetry?
The toppling rule is symmetric: each topple distributes to all 4 cardinal neighbours equally. The drop point is the exact centre. Therefore the pile always maintains perfect 4-fold rotational and reflective symmetry. The fractal structure is a consequence of how binary representations of the grain counts interfere under repeated cascading.

## Key Constants

| Constant | Effect |
|---|---|
| DROPS_DEF | Grain drops per frame in instant mode (default 10) |
| DROPS_MAX | Maximum drops per frame (500) |
| QMAX | Avalanche queue capacity (MAX_ROWS × MAX_COLS + 1) |
| GRAIN_CH[4] | Characters for grain counts 0–3 |
| TICK_NS | Frame duration (~30 fps) |
