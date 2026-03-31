# Pass 1 — bonsai: Growing Bonsai Tree Animation

## Core Idea

A bonsai tree grows branch by branch in the terminal, one step per tick. Each branch is a struct with a position, direction, and remaining life. At each tick, every live branch moves one cell, may spawn child branches, and draws its character. When a branch dies, it scatters leaf characters around its final position. The result looks like a tree growing in real time.

---

## The Mental Model

Think of the tree as a pool of "agents" (branches). You start with one trunk branch growing upward from the pot. Each tick, every live branch:
1. Picks its next direction (biased random walk, depends on tree type)
2. Moves one cell in that direction
3. Maybe spawns two child branches (left and right)
4. Decrements its life counter
5. Draws one character at its new position

When a branch's life hits zero, it dies and drops a few leaf characters nearby. New branches keep being added to the pool. When all branches are dead, the tree is done.

There is no grid/framebuffer — the tree is drawn by writing characters directly to `stdscr` as branches grow. The pot and static elements are redrawn every render frame on top of the growing tree.

---

## Data Structures

### `Branch` struct
| Field | Meaning |
|---|---|
| `x, y` | Current cell position |
| `life` | Steps remaining before this branch dies |
| `type` | BR_TRUNK / BR_LEFT / BR_RIGHT / BR_DYING / BR_DEAD |
| `dx, dy` | Current direction: each is −1, 0, or +1 |
| `age` | Steps taken so far (used for character selection and spawn timing) |
| `alive` | Whether this branch is still active |

### `Tree` struct
| Field | Meaning |
|---|---|
| `pool[BRANCH_MAX]` | Fixed array of all branches (alive + dead) |
| `n` | Number of branches allocated so far |
| `growing` | False when all branches dead |
| `shoots` | Count of live branches this generation |
| `shoot_max` | Budget limit on shoots (= multiplier) |
| `leaf_set` | Index into leaf character sets |
| `tree_type` | Enum controlling growth algorithm weights |
| `life_start` | Initial life given to trunk |
| `multiplier` | Branching density (0–20) |

### `Scene` struct
Owns Tree + all settings (tree_type, pot_type, multiplier, life, grow_fps, seed, modes).
Also tracks: `done`, `growing`, `wait_accum` (for infinite regrow delay).

---

## The Main Loop

**Two separate rates:**
- **Growth rate** (`grow_fps`): controls how many branch steps happen per second (default 30 steps/sec)
- **Render rate** (60 fps cap): how often the screen is redrawn

Each frame:
1. Compute `dt`
2. Add `dt` to `sim_accum` (growth accumulator)
3. While `sim_accum >= grow_tick_ns`:
   - Call `scene_tick()` → `tree_step()` advances every live branch one step, drawing directly to stdscr
   - Subtract `grow_tick_ns`
4. If tree is done and in infinite mode: wait 4 seconds then replant
5. `scene_draw_static()` — redraw pot, message box, HUD over the growing tree
6. `wnoutrefresh()` + `doupdate()` — flush

Notice: **no `erase()` before tree_step**.  The tree characters persist between frames because we want the drawn-so-far tree to stay visible as new branches grow. Only when replanting do we `erase()`.

---

## Non-Obvious Decisions

### Why no erase() during growth?
The tree grows incrementally — each branch draws one character per tick and we want to *accumulate* characters on screen to form the tree shape. Erasing would wipe the tree every frame. The static elements (pot, HUD) are redrawn every render frame on top of the growing characters.

### Branch character selection (`branch_char`)
Characters reflect the visual direction of each branch:
- Moving straight up → `|`
- Moving up-right → `\`, up-left → `/`
- Moving horizontal → `_`
- Dying branch → `~`
This makes the tree look like real wood at any angle.

### Branch color by age/life
- Young trunk (lots of life left) → dark brown (pair 1, bold) — thick base
- Older trunk + thick branches → light amber (pair 2)
- Dying branches → light wood, no bold — thinner appearance

### Child branch spawning
Two things trigger branching:
1. **Periodic**: every `multiplier+1` trunk steps
2. **Random**: 1-in-`(MULT_MAX - multiplier + 2)` chance each step

Both conditions only fire if trunk is more than 3 rows above the pot (no low branching). Children get `life / 2` — half the parent's remaining life. Always spawned in pairs (LEFT + RIGHT) to create symmetric branching.

### `next_dir` per tree type
Each tree type has a different biased random walk:
- **BAMBOO**: trunk almost always straight up (1-in-10 drift); branches spread then flatten
- **WEEPING**: branches droop downward after a few steps (dy goes +1)
- **DWARF**: dense horizontal spread
- **SPARSE**: branches stay upward, low drift
- **RANDOM**: cbonsai default — balanced upward bias with moderate spread

### ACS box-drawing characters for message box
`ACS_ULCORNER`, `ACS_HLINE`, `ACS_VLINE`, etc. are ncurses virtual characters that map to the terminal's line-drawing charset. They produce proper box corners without depending on Unicode.

---

## State Machines

### Tree growth state
```
[PLANTING]
    │
    ▼
[GROWING] ← tree_step() returns true each tick
    │
    └── all branches dead → tree_step() returns false
    │
    ▼
[DONE]
    │
    ├── infinite=false → stay done, show final tree
    │
    └── infinite=true → wait 4 seconds
                           │
                           ▼
                        [PLANTING] (erase + replant)
```

### Branch life state
```
[alive, type=TRUNK/LEFT/RIGHT]
    │
    ├── life > 3 → draw normal character
    └── life <= 3 → type = BR_DYING, draw '~'
    │
    └── life == 0
         ├── alive = false, type = BR_DEAD
         └── scatter 3-6 leaf characters nearby
```

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|---|---|---|
| `LIFE_DEFAULT` | 120 | Higher = taller, more complex tree |
| `MULT_DEFAULT` | 8 | Higher = more branches (denser canopy) |
| `GROW_FPS_DEFAULT` | 30 | Higher = faster growth animation |
| `BRANCH_MAX` | 1024 | Max simultaneous branches; overflow silently stops adding |
| `INFINITE_WAIT_MS` | 4000 | Pause between trees in infinite mode |

---

## Open Questions for Pass 3

1. What happens visually if children always get full parent life instead of half?
2. Can you reproduce the weeping effect by just changing `next_dir` for one type?
3. Why does the trunk not branch in the bottom 3 rows? What would it look like without this guard?
4. The pot is redrawn every render frame — why doesn't it overwrite the tree? (Think about draw order and what's in the scene_draw_static.)
5. What would change if you added `erase()` at the start of each growth tick?
6. How would you add a new tree type (e.g., "spiral" that rotates clockwise)?
