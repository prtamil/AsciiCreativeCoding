# Pass 1 — artistic/wator.c: Wa-Tor Predator-Prey Simulation

## Core Idea

Wa-Tor is a toroidal planet of water, populated by fish and sharks. Fish breed and drift randomly. Sharks hunt fish to survive; starve if they can't eat. The interaction produces emergent population cycles — boom-and-bust oscillations that appear, stabilise, and collapse depending on the breeding and starvation parameters. The grid is a discrete-time cellular automaton; the emergent dynamics are continuous and complex.

## The Mental Model

Picture a coral reef as a grid. Each cell is either empty, contains a fish, or contains a shark. Every tick, every creature acts: fish wander to empty neighbors and breed if old enough; sharks hunt adjacent fish, breed if well-fed, die if starving. No global coordination — every pattern emerges from these three local rules applied simultaneously.

The simulation is "Lotka-Volterra on a grid": fish population grows unchecked until sharks multiply and crash it. Then sharks starve, fish recover, and the cycle repeats. With the right parameters you can lock this oscillation into a stable limit cycle visible as alternating cyan/red waves in the population histogram.

## Grid State

Four `uint8_t` arrays, one per cell property:

```c
g_type  [r][c]  — EMPTY / FISH / SHARK
g_breed [r][c]  — fish: age since last birth; shark: breed age counter
g_hunger[r][c]  — shark: ticks since last meal (fish eat resets to 0)
g_moved [r][c]  — 1 if this cell's entity already acted this tick
```

`g_moved` is the critical deduplication guard. When a shark moves to (nr,nc) and eats the fish there, `g_moved[nr][nc] = 1` prevents the shark from being processed again when the shuffled order reaches that cell later in the same tick.

## Processing Order — Fisher-Yates Shuffle

```c
static int g_order[MAX_ROWS * MAX_COLS];

static void ishuffle(int *a, int n) {
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int tmp = a[i]; a[i] = a[j]; a[j] = tmp;
    }
}
```

Every tick, all live cell indices are shuffled before processing. Without shuffling, fish always move right/down preferentially (top-left processing order), creating directional drift patterns that look artificial. The shuffle makes movement isotropic.

## Fish Step

```c
age++; find random empty neighbor;
if (found) {
    move to neighbor; g_moved[neighbor] = 1;
    if (age_at_origin >= FISH_BREED) {
        leave baby fish at origin; reset origin breed counter;
    }
}
```

Fish only breed when old enough (`FISH_BREED = 3` ticks). Breeding leaves a new fish at the departure cell rather than the arrival cell — this ensures the newborn doesn't get processed in the same tick.

## Shark Step

```c
breed++; hunger++;
if (hunger > SHARK_STARVE) die (set EMPTY);
else {
    find random fish neighbor;
    if (found) { eat fish; move there; g_moved[there] = 1; hunger = 0; }
    else        { find random empty neighbor; move if found; }
    if (breed >= SHARK_BREED) { breed child at origin; reset breed counter; }
}
```

Sharks check for fish neighbors before empty neighbors — they always eat when possible. `SHARK_STARVE = 4` means a shark that can't find food for 4 ticks dies. This is the parameter most sensitive to oscillation stability.

## Dual Population Histogram

Four rows at the bottom split into two 2-row panels:
- Upper 2 rows: fish population (cyan), scale = max_pop / 2
- Lower 2 rows: shark population (red), scale = max_pop / 10

The shark scale is 5× compressed relative to fish because sharks are fewer in number. The ring buffer stores the last `g_cols` population samples for scrolling history.

## Non-Obvious Decisions

### Why move children to the parent's origin, not destination?
If the child appeared at the destination cell, it would need `g_moved[dest] = 1` for the parent. But we'd also need to check whether the new child has already been assigned `g_moved` status. Placing the child at the origin side-steps this: the origin cell was already processed (the parent just left it), so `g_moved[origin]` is guaranteed to be stale from the previous tick. The child won't act again this tick.

### Why shuffle all indices, not just live-entity indices?
Shuffling all `rows × cols` indices and skipping EMPTY cells is simpler and equally correct. The overhead is O(n) where n = total cells, not just live cells — but at 30fps with a 128×320 grid, this is 40,000 swaps per frame, negligible.

### Why are FISH_BREED and SHARK_STARVE the most important constants?
- `FISH_BREED` controls how fast fish reproduce. Too low → explosion. Too high → extinction.
- `SHARK_STARVE` controls predator pressure. Too low (lenient) → fish survive, sharks dominate, fish collapse, sharks collapse. Too high (strict) → sharks die before cycling begins.
- The interesting dynamics live in a narrow parameter band. The defaults (`FISH_BREED=3`, `SHARK_STARVE=4`) produce visible oscillations in most terminal sizes.

## From the Source

**Algorithm:** Wa-Tor predator-prey cellular automaton (Dewdney, A.K., "Computer Recreations", Scientific American, 1984). Shuffled update uses Fisher-Yates on the active cell list — equivalent to random-order asynchronous update, which better models spatial competition than top-left sequential scan.

**Physics:** Lotka-Volterra dynamics emerge: fish and shark populations oscillate with a phase lag (fish peak before shark peak). Parameter space: low FISH_BREED + high SHARK_BREED → shark overpopulation and collapse; opposite → shark starvation. Stable cycles exist in a narrow band around FISH_BREED=3, SHARK_STARVE=4.

**Data-structure:** Grid stored as parallel `uint8_t` arrays: `g_type[r][c]`, `g_breed[r][c]`, `g_hunger[r][c]`, `g_moved[r][c]`. Active cell array rebuilt O(rows×cols) each tick; Fisher-Yates shuffle is O(N_active). Population history ring buffer HIST_LEN=512 for the dual bar chart. MAX_ROWS=128, MAX_COLS=320.

## Key Constants

| Constant | Effect |
|---|---|
| FISH_BREED | Ticks before a fish can breed |
| SHARK_BREED | Ticks before a shark can breed |
| SHARK_STARVE | Ticks without food before a shark dies |
| FISH_INIT_PCT | Initial grid density of fish (%) |
| SHARK_INIT_PCT | Initial grid density of sharks (%) |
