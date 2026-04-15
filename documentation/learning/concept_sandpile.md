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

## View Modes (s / S key)

Three display modes cycle with the `s`/`S` key:

- **SPLIT** (default): top 60% shows the top-down grid view, a separator line, bottom 40% shows the side profile histogram
- **TOP**: full screen top-down grid — fractal pattern in full resolution
- **SIDE**: full screen side histogram — see the grain distribution across all columns

### Side View — Fixed-Scale Histogram
Each bar in the side view represents the total grain count summed down all rows of that column. The bar height is `col_sum * view_height / max_possible` where `max_possible = g_ca_rows * 3` (fixed scale — maximum possible sum if every cell holds 3 grains). This fixed scale is critical: if the scale were dynamic (normalised to the current max column sum), the bars would look the same shape even as the pile grows. With the fixed scale, bars visually grow from zero as grains accumulate, giving genuine feedback on pile growth.

## From the Source

**Algorithm:** Abelian Sandpile Model (Bak, Tang & Wiesenfeld, 1987). A grain is dropped on a chosen cell. If any cell has ≥ 4 grains, it "topples": loses 4 grains and gives 1 to each of its 4 (von Neumann) neighbours. Toppling can cascade into an "avalanche" affecting many cells.

**Math:** The sandpile is a canonical example of Self-Organised Criticality (SOC): without any tuning, the system spontaneously evolves to a critical state where avalanche size distributions follow a power law `P(s) ∝ s^(−τ)` with τ ≈ 1.0 for 2D abelian sandpile. "Abelian" means the final state is independent of the order in which topplings are processed — a deep mathematical property proved by Dhar (1990).

**Performance:** O(avalanche_size) per grain drop. Average avalanche size grows as the pile approaches the critical state. At criticality, large avalanches are rare but unbounded — the expected cost per drop diverges logarithmically. DROPS_DEF=10 grain drops per frame (default), DROPS_MAX=500 maximum.

**References:** Bak, Tang & Wiesenfeld (1987) — original SOC / sandpile paper; Dhar (1990) — proof of the abelian property.

## Themes (t key)

10 vivid themes, each with 4 content color pairs (G1, G2, G3, TOPPLE) plus HUD. All colors are chosen from bright regions of the xterm-256 palette — no dark or muted tones. Themes: Electric (39/51/226/201), Matrix (28/46/118/82), Nova (21/39/117/231), Poison (100/148/190/82), Ocean (24/38/45/159), Fire (196/208/226/231), Gold (136/178/220/231), Ice (30/45/159/231), Nebula (93/141/183/231), Lava (124/196/214/226).
