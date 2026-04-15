# Pass 1 — forest_fire.c: Drossel-Schwabl Forest Fire CA

## Core Idea

A 2D grid where every cell is one of three states — EMPTY, TREE, or FIRE — updated simultaneously each tick by three probabilistic rules:

1. **Fire burns out:** FIRE → EMPTY (deterministic, one tick)
2. **Fire spreads:** TREE → FIRE if any orthogonal neighbour is FIRE (deterministic)
3. **Lightning ignites:** TREE → FIRE with probability `f` (stochastic)
4. **Regrowth:** EMPTY → TREE with probability `p` (stochastic)

The two parameters `p` and `f` are the whole model. Their ratio `p/f` determines the character of the forest: the distribution of fire sizes, the density of the canopy, and whether fires are frequent-small or rare-catastrophic.

Reference: Drossel & Schwabl, *Physical Review Letters* 69(11):1629–1632, 1992.

## The Mental Model

### Why this is not just a random process

At first glance, TREE→FIRE with probability `f` looks like uncorrelated random sampling. But the state of each cell depends on its neighbours (fire spread), and the neighbours' states depend on their neighbours, and so on. The correlations propagate across the grid and across time. The system develops spatial structure (clusters) and temporal structure (rare large events).

### The p/f ratio and cluster size

Imagine `f = 0` (no lightning). Trees only burn if a neighbour burns. Starting from a seed fire, the fire can only spread as far as the connected cluster of trees. After it burns out, that region regrows (slowly, with probability `p` per tick). At steady state, the cluster size distribution depends on how long trees grow before the next fire visits — which is controlled by `p/f`.

- High `p/f`: trees grow back quickly, form large clusters → fires can spread very far → occasional system-spanning fires
- Low `p/f`: trees grow back slowly, clusters stay small → fires exhaust their fuel quickly → many small isolated fires

### Self-organised criticality (SOC)

At a specific ratio (roughly `p/f ≈ 200` for this model), the system sits at the **critical point** of a phase transition. At this point:

- The cluster size distribution is a power law: `P(s) ∝ s^{−τ}`, `τ ≈ 1.19`
- There is no characteristic fire size — fires at all scales coexist
- The system reaches this state automatically without tuning (self-organised)

This is the same phenomenon Bak, Tang, and Wiesenfeld called **self-organised criticality (SOC)** in their 1987 sandpile model. Real forests, real earthquakes, and real financial markets all show similar scale-free event size distributions, suggesting they operate near SOC.

**Why does it self-organise?** If `p` is too small relative to `f`, fires quickly outstrip regrowth and the forest stays sparse — fires can't spread far. If `p` is too large, dense clusters form and large fires return frequently — but fires keep resetting the density. The two forces (growth and destruction) balance at the critical density automatically.

### Double-buffer update

All cells must update simultaneously. If you updated left-to-right, a FIRE at column 5 that becomes EMPTY would no longer spread to column 6 — but the rule says fire spreads to any neighbour that is currently FIRE. The double-buffer `g_next` stores next states without overwriting `g_grid` mid-sweep. After all cells are computed: `memcpy(g_grid, g_next)`.

This is identical to Conway's Game of Life's two-grid approach.

### 4-neighbour vs 8-neighbour spread (preset 3)

With 4-neighbour spread, fire propagates in a diamond shape — fastest along axes, never diagonal. With 8-neighbour spread, fire can move diagonally too, producing rounder fronts and a higher effective contagion rate. The same `f` and `p` produce larger fires in 8-neighbour mode because each burning cell has up to 8 possible targets instead of 4.

## Data Structures

### Grids (§4)
```c
uint8_t g_grid[ROWS_MAX][COLS_MAX]   // current state: EMPTY=0, TREE=1, FIRE=2
uint8_t g_next[ROWS_MAX][COLS_MAX]   // next state (double-buffer)
uint8_t g_ash [ROWS_MAX][COLS_MAX]   // 1 if cell burned this tick (for ash display)
```

### Preset (§4)
```c
typedef struct {
    float p_grow;        // growth probability
    float p_fire;        // lightning probability
    bool  eight_neighbor;// diagonal spread?
    float density;       // initial tree fraction
    const char *name;
} Preset;
```

No per-cell agent struct — the grid IS the state.

## The Main Loop

1. **`grid_step()`**: iterate all `(r, c)`:
   - FIRE → set `g_next[r][c] = EMPTY`, mark `g_ash[r][c] = 1`
   - TREE → check 4 (or 8) neighbours; if any is FIRE or `rand() < f`: set FIRE; else keep TREE
   - EMPTY → if `rand() < p`: set TREE; else keep EMPTY
   - After full sweep: `memcpy(g_grid, g_next)`

2. **`scene_draw()`**: for each cell, draw character and color based on state; ash cells show `'.'` for one tick

## Non-Obvious Decisions

### xorshift32 RNG instead of `rand()`

The inner loop calls the RNG once per cell per tick. With a 80×24 terminal that's ~1920 calls/tick at 20fps = ~38,400 calls/second. `rand()` with `RAND_MAX=32767` (common on some platforms) would only give 15 bits of precision, and `rand()` itself can be slow due to global state and locking. The xorshift32:
```c
g_rng ^= g_rng << 13;
g_rng ^= g_rng >> 17;
g_rng ^= g_rng << 5;
```
generates 32 bits per call, passes the BigCrush randomness suite for this use case, and is 3 XOR+shift operations — faster than a `rand()` call with modulo.

Float conversion: `(rng_next() >> 8) / (1 << 24)` gives uniform float in [0,1) with 24 bits of precision (more than enough for probability comparisons).

### Ash display: `g_ash` is separate from `g_next`

When FIRE → EMPTY, we want to show ash `'.'` for one tick. But `g_next[r][c] = EMPTY` overwrites the FIRE state immediately — the draw pass can't tell that this cell just burned. Solution: a separate boolean `g_ash[r][c]` set whenever FIRE → EMPTY. It's reset to 0 at the start of each `grid_step()`, so it only persists for one draw frame.

### Flickering fire without per-cell randomness

Drawing `'*'` or `','` by `(r + c + tick) & 1` creates a checkerboard pattern that shifts by one cell each tick. To the eye at real-time framerates, this looks like random flickering but costs zero RNG calls at draw time and never repeats the same pattern in consecutive frames.

### Manual scatter (`s` key)

Igniting `SCATTER_COUNT=12` random TREE cells simultaneously creates an instant stress test: how does the forest respond to multiple simultaneous ignitions? This lets you empirically observe:
- Which fires die quickly (isolated trees, no cluster)
- Which fires cascade (edge of a dense cluster)
- Whether the fires merge into one large front or stay independent

### Probability clamping

`g_p_grow` and `g_p_fire` are adjusted via `g/G` and `l/L` keys. Clamped to [P_GROW_MIN, P_GROW_MAX] and [P_FIRE_MIN, P_FIRE_MAX] to prevent degenerate states (p=0 means no regrowth, f=0 means no fire — both result in a static grid).

## State Machine

```
PRESETS 0..3 — no automatic transitions; n/N to switch
  Each preset sets: p_grow, p_fire, eight_neighbor, density
  Switching preset resets the grid (scene_init)

PER TICK:
  grid_step():
    for all cells simultaneously:
      FIRE  → EMPTY + ash mark
      TREE  → FIRE (spread) | FIRE (lightning, prob f) | TREE
      EMPTY → TREE (prob p) | EMPTY
  scene_draw():
    FIRE alternate '*'/',' by (r+c+tick)&1
    EMPTY with ash mark → '.'
    TREE → '^'
```

## From the Source

**Algorithm:** Drossel-Schwabl (1992) three-state synchronous CA. Double-buffer pattern: all next-gen states computed from current gen before any are applied — identical to Conway GoL.

**Physics:** Self-organised criticality (Bak, Tang, Wiesenfeld 1987): at the critical ratio p/f, fire cluster sizes follow `P(s) ∝ s^(−τ)`, τ ≈ 1.19. Analogous to earthquakes (Gutenberg-Richter law) and real forest fires — no characteristic scale, system self-tunes to critical point without external parameter adjustment.

**Math:** Spread kernel choice: 4-neighbour (von Neumann neighbourhood) vs 8-neighbour (Moore neighbourhood). Moore neighbourhood raises effective contagion rate and changes fire-front isotropy; same p/f produces larger fires in 8-neighbour mode.

**Performance:** O(rows × cols) per tick. Per-cell RNG uses xorshift32 (3 XOR+shift ops) — faster than `rand()` and avoids platform-specific RAND_MAX=32767 precision limits. Float conversion: `(rng >> 8) / (1 << 24)` gives 24-bit uniform [0, 1).

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `P_GROW_DEF` | 0.030 | Higher → denser forest, larger clusters, rarer but bigger fires |
| `P_FIRE_DEF` | 0.0002 | Higher → more lightning, smaller clusters, frequent small fires |
| `SCATTER_COUNT` | 12 | More manual ignitions per `s` press |
| `SIM_FPS_DEF` | 20 | Lower → each tick visible; higher → watch time-lapse evolution |

## Themes

5 themes cycle with `t`/`T`:

| # | Name | EMPTY | TREE | FIRE | Mood |
|---|---|---|---|---|---|
| 0 | Classic | Dark grey | Green | Orange/red | Standard ecological |
| 1 | Night | Near-black | Dark green | Yellow | Night fire |
| 2 | Autumn | Dark grey | Orange | Red | Autumn forest |
| 3 | Boreal | Very dark | Cyan-tinted | Yellow-white | Northern boreal |
| 4 | Lava | Near-black | Dark green | Magenta | Volcanic |

---

# Pass 2 — forest_fire: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | EMPTY/TREE/FIRE states, P_GROW_DEF, P_FIRE_DEF, probability step/min/max |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5 themes × 6 color pairs (empty/tree/fire1/fire2/ash/hud) |
| §4 grid | `g_grid`, `g_next`, `g_ash`, `grid_step()`, `grid_seed()`, `grid_scatter()` |
| §5 scene | `scene_draw()`, `scene_init()` |
| §6 screen | ncurses init, `screen_resize()` |
| §7 app | main loop, input, signal handlers |

## Data Flow

```
grid_step():
  memset(g_ash, 0)
  for r in 0..g_rows-1:
    for c in 0..g_cols-1:
      state = g_grid[r][c]
      if state == FIRE:
        g_next[r][c] = EMPTY
        g_ash[r][c]  = 1
      elif state == TREE:
        neighbor_fire = any of {(r-1,c),(r+1,c),(r,c-1),(r,c+1)} == FIRE
        if eight_neighbor: also check diagonals
        if neighbor_fire or rng_float() < g_p_fire:
          g_next[r][c] = FIRE
        else:
          g_next[r][c] = TREE
      else: /* EMPTY */
        g_next[r][c] = (rng_float() < g_p_grow) ? TREE : EMPTY
  memcpy(g_grid, g_next, g_rows * COLS_MAX)
  update g_n_tree, g_n_fire, g_n_empty counts

scene_draw():
  for each cell (r, c):
    if g_grid[r][c] == EMPTY:
      if g_ash[r][c]: draw '.' CP_ASH
      else:           draw ' ' CP_EMPTY
    elif g_grid[r][c] == TREE:
      draw '^' CP_TREE A_BOLD
    else: /* FIRE */
      bright = (r + c + tick) & 1
      if bright: draw '*' CP_FIRE2 A_BOLD
      else:      draw ',' CP_FIRE1
  draw HUD: tree%, fire%, p, f, preset, theme, tick
```

## Open Questions for Pass 3

- Measure the fire size distribution: log-log plot of frequency vs size. Does it show a straight line (power law) at the default p/f ratio?
- At what exact p/f does the distribution become power-law? Sweep p from 0.001 to 0.100 with f fixed.
- How does 8-neighbour spread change the power-law exponent τ? Theory predicts τ should change slightly because the local contagion geometry is different.
- Is there a percolation threshold? At what tree density does a fire always span the full grid?
- Add a second FIRE state (EMBER) that spreads with lower probability — does this create realistic smoldering front behaviour?
- The model has no wind. Adding a directional bias (higher spread probability in one direction) breaks the isotropy — does SOC persist with wind?
