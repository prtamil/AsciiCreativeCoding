# Pass 1 — artistic/langton.c: Langton's Ant + Turmites

## Core Idea

An ant walks on a grid of coloured cells. At each step, the ant looks at the colour of the cell it's standing on, uses the colour to decide which way to turn (left or right), advances the cell to the next colour, then moves one step forward. Starting from a uniform grid, the ant leaves trails that look random at first. With the classic "RL" (right-left) rule, after about 10,000 steps an orderly diagonal highway suddenly emerges from the chaos and the ant marches off to infinity.

## The Mental Model

Imagine an ant on a checkerboard where cells are either white or black. On white: turn right, colour the cell black, step forward. On black: turn left, colour the cell white, step forward. This is Langton's Rule RL. For the first 10,000 steps the pattern looks like random noise. Then suddenly the ant starts building a diagonal highway and never stops. Nobody fully understands why.

## Rule String Encoding

The rule is a string of `R` and `L` characters, one per cell state. The ant's behaviour on a cell in state `s`:

1. Read `rule[s % n_colors]` — 'R' or 'L'
2. Turn right (dir+1)%4 or left (dir+3)%4
3. Advance the cell: `state → (state + 1) % n_colors`
4. Move one step forward

With "RL" (2 states): state 0 → turn R, becomes state 1; state 1 → turn L, becomes state 0.  
With "LLRR" (4 states): four possible cell colors, each with its own turn direction. Much more complex patterns emerge.

## Ant Step Implementation

```c
int   state = g_grid[ant->r][ant->c];
char  turn  = g_rule[state % g_n_colors];
ant->dir    = (turn == 'R') ? (ant->dir + 1) % 4   // clockwise
                            : (ant->dir + 3) % 4;   // counter-clockwise
g_grid[ant->r][ant->c] = (state + 1) % g_n_colors;
ant->r = (ant->r + DR[ant->dir] + g_rows) % g_rows; // toroidal wrap
ant->c = (ant->c + DC[ant->dir] + g_cols) % g_cols;
```

Direction vectors: `DR = {-1, 0, 1, 0}`, `DC = {0, 1, 0, -1}` for N/E/S/W.

## Eight Presets

| Rule | Name | Behavior |
|---|---|---|
| RL | Langton RL | Highway emerges after ~10,000 steps |
| LR | Fractal | Symmetric fractal-like growth |
| LLRR | Square spiral | Growing square spiral |
| RLR | Chaotic | Chaotic, no highway |
| LRRL | Complex | Complex branching structures |
| RRLL | Symmetric | Symmetric highway variant |
| RLLR | Tiling | Tiling-like repeating pattern |
| LLRRR | Irregular | Irregular growth |

## Multiple Ants

1–3 ants share the same grid. They interact because each ant reads and modifies cell states that other ants have also visited. This produces emergent structures not seen with a single ant — the ants' trails interfere, reinforce, or cancel each other.

Starting positions spread across the grid at fractions (0.5, 0.5), (0.35, 0.35), (0.65, 0.65) of the terminal dimensions.

## Speed

`STEPS_DEF = 200` ant steps per frame. The `+`/`-` keys double/halve the step count (up to `STEPS_MAX = 2000`). This is essential for "RL" where the highway only appears after ~10,000 total steps — at the default 200 steps/frame it takes ~50 frames (~1.7 s) to reach the highway; at 2000 it takes ~5 frames (~0.17 s).

## Non-Obvious Decisions

### Why high default step count vs other programs?
The most famous result (the RL highway) requires 10,000+ steps. A step count of 1 would take 5+ minutes to see the highway. The default of 200 shows the highway in seconds while still being slow enough to see the chaotic phase first.

### Why modulo 4 for direction?
Four directions (N/E/S/W) map cleanly to 0–3. Turn right: `+1`. Turn left: `−1` which is `+3` mod 4. No conditionals needed for wrap-around.

### Why toroidal grid?
Without wrap-around, ants eventually walk off the edge and the program would need boundary handling. Toroidal topology means ants run forever and trails wrap around, producing interesting patterns when ants re-enter the area they've previously visited from the other side.

### Why colour cells rather than just flip?
Multi-colour rules ("LLRR", "LRRL" etc.) produce much richer patterns than two-state rules. The generalisation to n-colour turmites was discovered independently after Langton's original work and produces qualitatively different emergent structures.

## From the Source

**Algorithm:** Langton's Ant — a 2-D Turing machine on a grid. Generalised multi-colour variant: rule string of length K means K colours; rule[i] is the turn direction for colour i.

**Math:** The ant's state space is (position, heading, grid) — theoretically infinite but bounded by toroidal wraparound. The system is deterministic and reversible (time-reversal = swap R↔L in rule). The RL highway has period = 104 steps.

**Physics:** Emergent order from local rules: with rule "RL", the ant appears chaotic for ~10,000 steps, then spontaneously builds a periodic "highway" — a striking example of emergence: complex global structure from simple local rules.

**Performance:** O(steps_per_frame) per tick; grid update is O(1) per step.

## Key Constants

| Constant | Effect |
|---|---|
| STEPS_DEF | Default steps per frame; too low → miss the highway; too high → miss the chaotic phase |
| STEPS_MAX | Step count ceiling for + key |
| MAX_COLORS | Maximum cell states (8); limits rule string length |
| MAX_ANTS | Maximum simultaneous ants (3) |

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_grid[MAX_ROWS][MAX_COLS]` | `uint8[128][320]` | ~40 KB | cell-state grid (0 = colour 0, 1..N-1 = other colours) |
| `g_ants[MAX_ANTS]` | `Ant[3]` (row, col, dir) | ~36 B | position and heading of each ant |
| `g_rule` | `const char *` | 8 B | rule string (e.g. "RL") encoding turn direction per colour |
| `g_total_steps` | `long long` | 8 B | cumulative steps taken since last reset |
