# Pass 1 — forest_fire.c: Drossel-Schwabl Forest Fire CA

## Core Idea

A 2D grid where every cell is one of three states — EMPTY, TREE, or
FIRE — updated simultaneously each tick by four probabilistic rules:

1. **Fire burns out:** FIRE → EMPTY (deterministic, one tick)
2. **Fire spreads:** TREE → FIRE if any neighbour is FIRE (deterministic)
3. **Lightning ignites:** TREE → FIRE with probability `f` (stochastic)
4. **Regrowth:** EMPTY → TREE with probability `p` (stochastic)

The two parameters `p` and `f` are the entire model. Their ratio `p/f`
determines whether the forest is sparse-and-burning-often or dense-and-
catastrophic, and whether fire-size statistics follow a power law.

Reference: Drossel & Schwabl, *Physical Review Letters* 69(11):1629–1632,
1992.

## Mental Model

### Why this is not just a random process

At first glance, TREE→FIRE with probability `f` looks like uncorrelated
random sampling. But each cell's fate depends on its neighbours (fire
spread), and the neighbours' states depend on theirs, and so on. The
correlations propagate across the grid and across time. The system
develops spatial structure (clusters) and temporal structure (rare
large events).

### The p/f ratio and cluster size

Imagine `f = 0` (no lightning). Trees only burn when a neighbour
burns. Starting from a seed fire, the fire spreads through whatever
connected cluster of trees it touches. After it burns out, that
region regrows slowly (probability `p` per tick). At steady state the
cluster size distribution depends on how long trees grow before the
next fire visits — controlled by `p/f`.

- High `p/f`: trees grow back quickly, form large clusters → fires
  spread very far → occasional system-spanning fires.
- Low `p/f`: trees grow back slowly, clusters stay small → fires
  exhaust their fuel quickly → many small isolated fires.

### Self-organised criticality (SOC)

At a specific ratio (roughly `p/f ≈ 200` for this model), the system
sits at the **critical point** of a phase transition:

- Cluster size distribution is a power law `P(s) ∝ s^{−τ}`, τ ≈ 1.19.
- No characteristic fire size — fires at all scales coexist.
- The system reaches this state automatically without tuning.

This is the same phenomenon Bak, Tang, and Wiesenfeld called **self-
organised criticality** in their 1987 sandpile model. Real forests,
real earthquakes, and real markets all show similar scale-free event
size distributions.

**Why it self-organises:** if `p` is too small relative to `f`, fires
outstrip regrowth and the forest stays sparse — fires can't spread
far. If `p` is too large, dense clusters form and large fires reset
the density. The two forces balance at the critical density without
any external tuning.

## Worked Example (defaults: 80×24 terminal, p=0.030, f=0.0002)

- Per tick the inner loop fires ~1920 RNG calls (one per cell).
- At `p_grow = 0.030`, an empty cell reaches 50% TREE probability after
  ln(2)/0.030 ≈ 23 ticks. So at SIM_FPS=20 a freshly burned patch
  takes about 1.2 s to half-recover.
- At `p_fire = 0.0002`, the expected number of lightning strikes per
  tick across the grid is 1920 · 0.0002 ≈ 0.4. Roughly one strike
  every 2.5 ticks during steady-state.
- p/f = 150 — close to the critical ratio. Press `s` to scatter 12
  ignitions and watch which die quickly (isolated trees) and which
  cascade (cluster edges).

## Double-Buffer Update

All cells must update simultaneously. If the sweep updated cells
left-to-right, a FIRE at column 5 that becomes EMPTY would no longer
spread to column 6 — but the rule says fire spreads to any cell
currently on fire. Solution: a separate `g_next` array stores next-
generation states without overwriting `g_grid`. After the sweep:
`memcpy(g_grid, g_next)`.

This is identical to Conway's Game of Life's two-grid approach.

## Ash Overlay (the third buffer)

When FIRE → EMPTY, we want to show ash `'.'` for one tick before
the cell goes fully empty. But `g_next[r][c] = EMPTY` overwrites the
FIRE state immediately — the draw pass can't tell that this cell just
burned. Solution: a separate flag `g_ash[r][c]` set whenever
FIRE → EMPTY. It's reset to 0 at the start of each `grid_step()`,
so it persists for exactly one draw frame.

Visually this gives the forest a "burned-scar memory" — a 1-tick echo
that turns the fire front into a moving wave, not just a colour
change.

## Flickering Without Per-Cell RNG

Drawing FIRE as `'*'` (CP_FIRE2 bright) or `','` (CP_FIRE1 dim) by
`(r + c + tick) & 1` creates a checkerboard pattern that shifts by
one cell each tick. To the eye at real-time framerates it looks like
random flickering but costs zero RNG calls at draw time and never
repeats the same pattern in consecutive frames.

## xorshift32 RNG

The inner loop calls the RNG once per cell per tick — ~1920 calls/tick
at 20 fps = ~38 400 calls/second. `rand()` with `RAND_MAX=32767` would
only give 15 bits, and `rand()` itself can be slow due to global state.
xorshift32:
```c
g_rng ^= g_rng << 13;
g_rng ^= g_rng >> 17;
g_rng ^= g_rng << 5;
```
generates 32 bits per call in three XOR+shift ops — faster than `rand()`
and platform-independent.

Float conversion: `(rng_next() >> 8) / (1 << 24)` gives uniform float
in [0, 1) with 24 bits of precision (more than enough for probability
comparisons).

## 4-Neighbour vs 8-Neighbour Spread (Smoulder preset)

With 4-neighbour (von Neumann) spread, fire propagates in a diamond
shape — fastest along axes, never diagonal. With 8-neighbour (Moore)
spread, fire moves diagonally too, producing rounder fronts and a
higher effective contagion rate. Same `f` and `p` produce larger fires
in 8-neighbour mode because each burning cell has up to 8 possible
targets instead of 4.

The Smoulder preset (preset 3) enables 8-neighbour spread; presets
0–2 are 4-neighbour.

## Themes (5)

`t/T` cycles. All bg = -1 (terminal default), all fg in the bright
half of the 256-colour cube so A_DIM stays visible:

| # | Name    | EMPTY    | TREE      | FIRE          | Mood              |
|---|---------|----------|-----------|---------------|-------------------|
| 0 | Classic | dark grey| green     | orange/red    | Standard          |
| 1 | Night   | near-blk | dark green| yellow        | Night fire        |
| 2 | Autumn  | dark grey| orange    | red           | Autumn forest     |
| 3 | Boreal  | very dark| cyan-tint | yellow-white  | Northern boreal   |
| 4 | Lava    | near-blk | dark green| magenta       | Volcanic          |

PAIR_HUD (yellow 226 + A_BOLD) and PAIR_HINT (cyan 51 + A_BOLD) are
theme-independent and registered once in `color_init()`.

## Module Map

```
§1 config  — EMPTY/TREE/FIRE constants, P_GROW_DEF, P_FIRE_DEF,
             SCATTER_COUNT, SIM_FPS_DEF, theme/preset enums
§2 clock   — clock_ns + clock_sleep_ns
§3 color   — N_THEMES Theme records + theme_apply +
             PAIR_HUD/PAIR_HINT registered once
§5 grid    — g_grid/g_next/g_ash + grid_seed + grid_step +
             grid_scatter + has_fire_neighbor helper
§6 scene   — mark_cell + draw_grid + draw_hud + scene_draw
§7 screen  — ncurses init/resize
§8 app     — signal handlers, main loop
```

## Pseudocode

```
grid_step():
  memset(ash, 0)
  for each (r, c):
    state = grid[r][c]
    if state == FIRE:
      next[r][c] = EMPTY; ash[r][c] = 1
    elif state == TREE:
      if has_fire_neighbor(r, c, eight) or rng_float() < p_fire:
        next[r][c] = FIRE
      else:
        next[r][c] = TREE
    else:  /* EMPTY */
      next[r][c] = (rng_float() < p_grow) ? TREE : EMPTY
    update tree/fire/empty counters
  memcpy(grid <- next)

scene_draw():
  draw_grid():
    for each (r, c):
      if grid[r][c] == EMPTY:
        if ash[r][c]: '.'  CP_ASH
        else:         ' '
      elif grid[r][c] == TREE:
        '^' CP_TREE A_BOLD
      else:  /* FIRE */
        bright = (r + c + tick) & 1
        bright ? '*' CP_FIRE2 A_BOLD : ',' CP_FIRE1
  draw_hud():
    top-right yellow A_BOLD: tree%, fire%, p, f, preset, theme, fps
    bottom-left cyan A_BOLD: key hints
```

## Edge Cases

- **Ash buffer reset.** Must clear `g_ash` at the START of `grid_step()`.
  Forgetting this leaves yesterday's ashes painted forever.

- **8-neighbour mode and percolation.** With 8-neighbour spread at high
  p_grow, fires sometimes percolate the entire grid in one tick.
  This is correct behaviour, but the visual loses its narrative; lower
  p_grow or use preset 0–2 for "fire-front" aesthetics.

- **Probability clamping.** g/G and l/L adjust p_grow / p_fire in fixed
  steps. Clamped to [P_GROW_MIN, P_GROW_MAX] and [P_FIRE_MIN,
  P_FIRE_MAX] so they can't become 0 (degenerate static grid) or 1
  (instant catastrophe).

- **A_DIM avoidance.** The grayscale strip 232–239 is forbidden by
  CLAUDE.md (invisible under A_DIM). All theme colours sit at ≥ 24
  in the cube and ≥ 240 in grayscale, so theme tints are visible
  even when A_DIM is applied.

- **Scatter respects current trees.** `grid_scatter()` only ignites
  cells whose state is TREE. Trying to scatter into an empty grid
  (no trees yet) silently does nothing — not a bug, but a "wait until
  there's something to burn" reminder.

## How to Verify

- After many ticks at default p/f, the forest stabilises around a
  steady-state tree fraction of ~0.4–0.5 (visible in the HUD percentage).
  Pushing p_grow up shifts steady-state higher; pushing p_fire up
  shifts it lower.

- Cluster size distribution is power-law-ish at default ratio. Hard to
  see by eye, but a histogram of fire areas across many ticks should
  show a long tail with no characteristic scale.

- Doubling grid dimensions roughly quadruples per-tick CPU cost
  (O(rows × cols)).

- Disable lightning (set p_fire=0 via lots of L presses) and watch
  the forest mature into a static green field — confirms that
  spreading is the only fire-source when lightning is off.

- Press `s` repeatedly: visual stress test of multiple simultaneous
  ignitions. Some die alone (isolated trees), some merge fronts.

## Open Questions

1. Measure the fire-size distribution: log-log plot of frequency vs
   size. Does it show a straight line at the default p/f ratio?
2. At what exact p/f does the distribution become power-law? Sweep
   p from 0.001 to 0.100 with f fixed.
3. How does 8-neighbour spread change the power-law exponent τ?
4. Add a directional wind bias (higher spread probability in one
   direction). Does SOC persist with anisotropy?
5. Add a second fire state (EMBER) that spreads with lower probability
   — does this create realistic smouldering front behaviour?

## References

- Drossel & Schwabl, "Self-organized critical forest-fire model",
  *Physical Review Letters* 69(11):1629–1632, 1992.
- Bak, Tang & Wiesenfeld, "Self-organized criticality: An explanation
  of 1/f noise", *Physical Review Letters* 59:381, 1987.
- Wikipedia, *Forest-fire model*,
  https://en.wikipedia.org/wiki/Forest-fire_model
- Christensen et al., "Self-organized critical forest-fire model:
  Mean-field theory and simulation results in 1 to 6 dimensions",
  *Physical Review Letters* 71:2737, 1993.
