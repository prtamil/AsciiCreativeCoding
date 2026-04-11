# Concept: 2D Cellular Automaton

## Pass 1 — Understanding

### Core Idea
A 2D grid of cells, each with a small integer state. Each step, every cell updates its state based on a rule applied to its neighborhood. Unlike 1D CA (single rule table), 2D CA rules are typically defined by specifying which neighbor-count values cause birth, survival, or death.

### Mental Model
Each cell is a pixel. Every tick, count how many of its 8 neighbors are "on." Apply the rule: if a cell is on and has 2–3 live neighbors, it survives; if off and has exactly 3 neighbors, it's born. That specific rule is Conway's Life. Changing those counts completely changes the behavior.

### Key Concepts
- **Moore neighborhood**: 8 cells surrounding (distance 1 in any direction)
- **Von Neumann neighborhood**: 4 orthogonal cells only
- **Totalistic rule**: state depends only on neighbor count sum, not arrangement
- **Outer totalistic**: depends on both own state and neighbor sum

### Common Rules (Birth/Survival notation B/S)
| Rule | Behavior |
|------|---------|
| B3/S23 | Conway Life — complex emergent patterns |
| B36/S23 | HighLife — replicators |
| B2/S | Seeds — explosive growth |
| B3/S12345 | Maze-like structures |
| B4678/S35678 | Coral growth |

### Data Structures
- `grid[H][W]`: uint8 (0=dead, 1=alive, or multi-state)
- Double buffer: compute into `next[H][W]`, then swap
- Toroidal wrap or bounded edges

### Non-Obvious Decisions
- **Double buffer essential**: Updating in-place means cells computed later in the scan see already-updated neighbors. Wrong results. Always read from old grid, write to new.
- **Toroidal wrap**: Wrap edges with `(x + W) % W` so every cell has the same 8 neighbors. Without this, edge behavior differs.
- **Multi-state rules**: Some rules use 3+ states. Example: "dying" state where a cell fades over several generations (Wireworld uses 4 states).
- **Encode birth/survival as bitmask**: `birth_mask = 0b001000000` means B3. Check: `if (birth_mask >> neighbor_count) & 1`.

### Key Constants
| Name | Role |
|------|------|
| BIRTH_MASK | which neighbor counts cause birth |
| SURVIVE_MASK | which neighbor counts allow survival |
| WRAP | toroidal (1) or bounded (0) |

### Open Questions
- Can you implement Wireworld (4-state electron simulation)?
- Find a glider pattern in Life. What is its period?
- Try B3/S23 with a random starting density — what density gives max complexity?

---

## Pass 2 — Implementation

### Pseudocode
```
step():
    for y in 0..H:
        for x in 0..W:
            count = 0
            for (dy,dx) in neighborhood:
                ny = (y+dy+H)%H; nx = (x+dx+W)%W
                count += grid[ny][nx]   # for binary states
            alive = grid[y][x]
            if alive:
                next[y][x] = (SURVIVE_MASK >> count) & 1
            else:
                next[y][x] = (BIRTH_MASK >> count) & 1
    swap(grid, next)

draw():
    for y in 0..H:
        for x in 0..W:
            if grid[y][x]: mvaddch(y+offset, x+offset, '█' or '#')
            else: mvaddch(y+offset, x+offset, ' ')
```

### Module Map
```
§1 config    — H, W, BIRTH_MASK, SURVIVE_MASK, WRAP
§2 init      — random_fill(density), clear(), pattern_load()
§3 step      — count_neighbors(), apply_rule(), swap buffers
§4 draw      — grid to screen, color by state
§5 app       — main loop, keys (step, pause, rule select, draw mode)
```

### Data Flow
```
grid[H][W] → count neighbors → apply rule → next[H][W]
swap → draw to screen → next frame
```
