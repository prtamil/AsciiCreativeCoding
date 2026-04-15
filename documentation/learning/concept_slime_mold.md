# Pass 1 — slime_mold.c: Physarum polycephalum via Jeff Jones (2010)

## From the Source

**Algorithm:** Agent-based simulation — emergent behaviour from local rules. The trail grid mediates indirect communication (stigmergy): agents respond to paths left by earlier agents. No global coordinator; network topology arises purely from the sense→rotate→move→deposit loop.

**Math:** Diffusion step is a lerp toward a 3×3 box average (trail' = lerp(trail, avg_3×3(trail), DIFFUSE_W)) — a discrete approximation of the heat equation (∂u/∂t = D·∇²u). Decay: trail' *= (1 − DECAY_RATE) per tick; without diffusion, trail lifetime ≈ 1/DECAY ticks. At DECAY=0.08: lifetime ≈ 12.5 ticks ≈ 0.4 s half-life at 30 fps.

**Performance:** O(N_AGENTS + W×H) per tick. Trail grid update is the bottleneck: 512×128 ≈ 65K cells × 9-neighbour sum ≈ 590K ops per tick. Agents cost N_AGENTS × ~15 ops each. Two float arrays (g_trail/g_buf) for double-buffering: agents deposit into g_trail; grid update reads g_trail → writes g_buf → copies back, preventing mid-tick positional feedback.

**Physics/References:** Jeff Jones (2010) — *Characteristics of Pattern Formation and Evolution in Approximations of Physarum Transport Networks*, Artificial Life 16(2), 127–153. The model captures near-optimal Steiner tree topology using only three sensor readings and a random tie-break rule.

---

## Core Idea

A colony of 2000 agents moves through a 2D grid, each following three simple rules:
1. **Sense** — sample the chemical trail at three positions ahead (forward-left, forward, forward-right)
2. **Rotate** — turn toward the strongest signal; random coin flip if tied
3. **Move & Deposit** — advance one cell, leave a trail deposit

In parallel, the trail grid **diffuses** (box blur) and **decays** (exponential fade) every tick. The combination of positive feedback (trails attract agents who reinforce them) and negative feedback (trails decay without traffic) drives the colony to self-organise into a sparse, branching network topology that empirically connects food sources via near-minimum Steiner trees.

The algorithm is from: Jones, J. (2010) *Characteristics of Pattern Formation and Evolution in Approximations of Physarum Transport Networks*. Artificial Life 16(2), 127–153.

## The Mental Model

### Why sense-rotate-move (not just gradient descent)?

Pure gradient descent would make every agent rush directly toward the nearest food. The result is a single highway — no branching, no redundancy. The 45° sensor angle means agents can detect trails to the side; they turn toward the best neighbour rather than toward the global maximum. This local, noisy hill-climbing creates a rich branching structure because agents at branch points genuinely compete for followers.

### Positive vs negative feedback

- **Positive (reinforcement):** More agents on a path → more deposit → more agents follow. Busy paths grow.
- **Negative (decay):** Paths with no traffic fade away. The colony prunes its own network.

The balance between `DEPOSIT` (how much agents lay down), `DECAY` (how fast it disappears), and `DIFFUSE_W` (how fast it spreads) determines the network character:
- High decay + low deposit → sparse, threadlike; network barely forms
- Low decay + high deposit → blob; no branching because trail is everywhere
- Tuned middle range → biological-looking dendritic network

### Why do Steiner-like trees emerge?

A Steiner minimum tree is the shortest network of line segments connecting a set of points (with junction points allowed anywhere, unlike a minimum spanning tree which must use the input points as junctions). Finding it exactly is NP-hard. The slime mold finds near-optimal solutions because:
- Short paths between food sources stabilise first (less trail to maintain)
- Long detours fade before they can be reinforced
- The diffusion step helps agents "smell" food sources from a distance, biasing the initial colony drift

### Double-buffered diffusion

Diffusion is a neighbourhood operation: each cell should read its neighbours' *current* values, not already-updated values. Writing results directly into the trail grid means cells near the top of the update loop read stale inputs while cells near the bottom read updated values — breaking spatial symmetry. The fix: write all diffused values into a scratch buffer `g_buf`, then `memcpy(g_trail, g_buf)`. This is the same pattern as Game of Life's "two-buffer" update.

## Data Structures

### Agent (§5)
```
float x, y    — position in cell coordinates (float for sub-cell precision)
float angle   — heading in radians
```

### Grid (§5)
```
float g_trail[ROWS_MAX][COLS_MAX]   — current trail concentration
float g_buf  [ROWS_MAX][COLS_MAX]   — diffusion scratch buffer
```

### FoodSource (§6)
```
float cx, cy   — position in pixel space
float radius   — influence radius for deposit bonus
```

## The Main Loop

1. **Agent tick** (`agents_tick`): for each agent, call `agent_step()`:
   - Compute 3 sensor positions with `trail_sample()` (bilinear interpolation between cells)
   - Compare FL, F, FR; rotate by ±ROTATE_ANGLE or hold
   - Move by STEP_SIZE; wrap at grid boundaries (toroidal)
   - Deposit: DEPOSIT × FOOD_BONUS if within FOOD_RADIUS of any food, else DEPOSIT

2. **Trail update** (`trail_update`): iterate all cells, compute 3×3 box average with wrap, blend into buf: `buf = trail*(1-W) + avg*W`; multiply by `(1-DECAY)` for fade; memcpy buf→trail

3. **Draw** (`scene_draw`): map trail intensity to one of 5 characters and 5 colour pairs; draw food source stars; draw HUD

## Non-Obvious Decisions

### Bilinear trail sampling

Agent positions are float; the grid is integer. `trail_sample(x, y)` interpolates between the 4 surrounding cells:
```c
int c0 = (int)x, r0 = (int)y;
float fx = x - c0, fy = y - r0;
return lerp(lerp(g_trail[r0][c0],   g_trail[r0][c0+1],   fx),
            lerp(g_trail[r0+1][c0], g_trail[r0+1][c0+1], fx), fy);
```
Without interpolation, agents on non-integer positions sample abruptly, creating a grid-aligned artefact in the network.

### Deposit is instantaneous, not spread

Some slime mold implementations deposit a Gaussian blob around the agent. The Jones model deposits only at the agent's exact position. The diffusion step spreads it. This keeps the two concerns separate: deposit controls signal strength; diffusion controls reach.

### Sensor distance vs sensor angle trade-off

- Small SENSOR_DIST → agents react only to immediate surroundings; network is tight, over-fitted to local trail peaks
- Large SENSOR_DIST → agents are influenced by far-away trail; network spans larger regions but may skip nearby paths
- Small SENSOR_ANGLE → agents can't distinguish left from right early enough; more straight-line movement
- Large SENSOR_ANGLE (e.g. 90°) → agents detect perpendicular trails and make sharp turns; network becomes chaotic

The Jones defaults (SENSOR_DIST=4, SENSOR_ANGLE=45°) produce the characteristic Physarum-like dendritic branching.

### Food bonus vs food floor

Two mechanisms attract agents to food:
1. **FOOD_BONUS (6×):** Each visit by an agent near food deposits 6× the normal amount. This makes food regions a strong attractor for passing agents.
2. **FOOD_MIN_TRAIL (30.0):** The trail at food cells is clamped to a minimum each tick, so even if no agents visit, food cells remain detectable. This provides a persistent beacon at long distances.

Without the floor, isolated food sources that haven't been found yet have zero trail, meaning no agent is attracted to them until one stumbles in by diffusion.

### Toroidal wrap

The grid wraps at all four edges. This avoids boundary effects (agents clumping at edges, trails concentrating near walls) and lets the colony form a clean periodic network topology. The wrap is applied in both the agent step and the 3×3 diffusion stencil.

## State Machine

```
4 PRESETS (cycled with n/N):
  0 Scatter  — agents uniformly random; triangle food pattern
  1 Ring     — agents on ring, pointing inward; central food
  2 Clusters — two agent groups; symmetric food
  3 Mesh     — grid-aligned agents; corner food

Per tick (regardless of preset):
  agents_tick()   → for each agent: sense, rotate, move, deposit
  trail_update()  → diffuse + decay (double-buffered)
  scene_draw()    → map trail to chars, draw food stars, HUD
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `N_AGENTS_DEF` | 2000 | More agents → denser network, faster convergence |
| `SENSOR_ANGLE` | π/4 | Wider → more meandering; narrower → straighter paths |
| `SENSOR_DIST` | 4.0 cells | Longer → more global routing; shorter → local only |
| `ROTATE_ANGLE` | π/4 | Larger → sharper turns; smaller → gradual curves |
| `STEP_SIZE` | 1.0 | Larger → agents cover more ground; skips cells |
| `DEPOSIT_DEF` | 5.0 | Higher → faster network formation; too high → blob |
| `DECAY_DEF` | 0.08 | Higher → faster pruning; too high → network collapses |
| `DIFFUSE_DEF` | 0.35 | Higher → wider reach; too high → uniform field |
| `FOOD_BONUS` | 6.0 | Higher → network forced onto food paths |
| `FOOD_MIN_TRAIL` | 30.0 | Higher → food cells always visible from far away |

## Themes

5 themes cycle with `t`/`T`:

| # | Name | Trail colors | Character set |
|---|---|---|---|
| 0 | Physarum | Yellow/amber gradient | ` . + x # @` |
| 1 | Cyan | Cyan gradient | ` . + x # @` |
| 2 | Neon | Magenta/pink | ` . + x # @` |
| 3 | Forest | Green gradient | ` . + x # @` |
| 4 | Lava | Orange/red | ` . + x # @` |

---

# Pass 2 — slime_mold: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_AGENTS=2000, SENSOR_*, ROTATE_*, STEP_SIZE, DEPOSIT, DECAY, DIFFUSE |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5 themes × 5 trail colour pairs + food pair |
| §4 coords | `px_to_cx()`, `px_to_cy()` |
| §5 physics | `Agent`, `trail_sample()`, `trail_deposit()`, `agent_step()`, `trail_update()` |
| §6 scene | Preset builders, `scene_init()`, `food_setup()` |
| §7 draw | `scene_draw()`, `hud_draw()` |
| §8 app | main loop, input, signal handlers |

## Data Flow

```
agents_tick():
  for each agent a:
    agent_step(a):
      sense:
        fl_x = a.x + cos(a.angle - SENSOR_ANGLE) * SENSOR_DIST
        fl_y = a.y + sin(a.angle - SENSOR_ANGLE) * SENSOR_DIST
        f_x  = a.x + cos(a.angle) * SENSOR_DIST
        f_y  = a.y + sin(a.angle) * SENSOR_DIST
        fr_x = a.x + cos(a.angle + SENSOR_ANGLE) * SENSOR_DIST
        fr_y = a.y + sin(a.angle + SENSOR_ANGLE) * SENSOR_DIST
        vFL = trail_sample(fl_x, fl_y)
        vF  = trail_sample(f_x,  f_y)
        vFR = trail_sample(fr_x, fr_y)
      rotate:
        if vF >= vFL and vF >= vFR: keep angle
        elif vFL > vFR:             angle -= ROTATE_ANGLE
        elif vFR > vFL:             angle += ROTATE_ANGLE
        else:                       angle += random choice(+1,-1) * ROTATE_ANGLE
      move:
        a.x += cos(a.angle) * STEP_SIZE
        a.y += sin(a.angle) * STEP_SIZE
        wrap a.x in [0, grid_cols), a.y in [0, grid_rows)
      deposit:
        dep = DEPOSIT * food_multiplier(a.x, a.y)
        trail_deposit(a.x, a.y, dep)

trail_update(diffuse_w, decay):
  retain = 1.0 - decay
  for r in 0..rows-1:
    for c in 0..cols-1:
      sum = 0
      for dr in -1..1, dc in -1..1:
        sum += g_trail[wrap_r(r+dr)][wrap_c(c+dc)]
      avg = sum / 9.0
      g_buf[r][c] = (g_trail[r][c]*(1-diffuse_w) + avg*diffuse_w) * retain
  memcpy(g_trail, g_buf)

scene_draw():
  for each cell (r, c):
    v = g_trail[r][c]
    determine character and color from v (5 intensity bands)
    mvaddch with appropriate color pair
  draw food source '*' markers
  draw HUD: agents, diffuse, decay, fps, preset, theme
```

## Open Questions for Pass 3

- Does the Physarum network actually minimise total path length? Measure: sum edge lengths and compare to MST and Steiner tree.
- At what decay rate does the network collapse? Is there a sharp phase transition (like Ising)?
- How does the ring preset differ from scatter? Does the inward-pointing initialisation produce a faster or denser central connection?
- What happens when SENSOR_ANGLE = π/2? Do agents become confused by perpendicular trails?
- Can a Fourier analysis of the trail grid detect the characteristic frequency of the network branches?
- The real Physarum oscillates its internal pressure rhythmically — does adding a periodic deposit pulse (DEPOSIT × (1 + sin(t))) change the network structure?
