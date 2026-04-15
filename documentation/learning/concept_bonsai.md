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

## From the Source

**Algorithm:** Stochastic recursive tree generation (cbonsai algorithm). Each branch step chooses direction using weighted random selection based on branch age ("life") and tree type. Branches spawn child branches when life crosses thresholds, producing natural-looking asymmetric growth without L-systems.

**Data-structure:** Branch pool — flat array of branch structs (position, life, type, age). Active branches are processed FIFO each tick. Pool size is bounded by `BRANCH_MAX` (1024) to prevent unbounded memory use during long-lived infinite-mode runs.

**Rendering:** Branch character selected from a lookup table based on branch direction and age; leaf characters scattered at branch tips when life falls below `LEAF_THRESH`. 256-colour palette with 8-colour fallback; pot drawn with box-drawing characters.

**Randomness:** Seeded with `time(NULL)` by default; `r` key re-seeds for a new tree. Seed is displayed in the HUD so interesting trees can be reproduced.

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

---

# Pass 2 — bonsai: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | Constants: LIFE, MULT, GROW_FPS, BRANCH_MAX, INFINITE_WAIT_MS, pot sizes |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 7 pairs: dark wood, light wood, dark leaf, light leaf, pot, message, HUD |
| §4 pot | `draw_pot(cx, rows, type)` — ASCII art pot at bottom center |
| §5 branch | `Branch` struct, `branch_char()`, `branch_color()`, `branch_bold()` |
| §6 tree | `Tree` struct, `tree_reset()`, `tree_new_branch()`, `next_dir()`, `tree_step()` |
| §7 message | `draw_message()` — bordered ACS box around message string |
| §8 scene | `Scene` owns Tree + settings; `scene_plant()`, `scene_tick()`, `scene_draw_static()` |
| §9 screen | ncurses init/resize/present |
| §10 app | Main dt loop, input, signal handling |

---

## Data Flow Diagram

```
clock_ns() → dt
    │
    ▼
grow_accum += dt
    │
    while grow_accum >= grow_tick_ns:
    │
    ▼
tree_step(tree, cols, rows, trunk_base_y)
    │
    for each live branch in pool[0..n-1]:
    │
    ├── next_dir(tree_type, b.type, b.age, b.life, b.dx, b.dy)
    │       → new dx, dy   [biased random walk per tree type]
    │
    ├── b.x += dx, b.y += dy, b.age++, b.life--
    │
    ├── branch_char(dx, dy, type, age) → ch
    ├── branch_color(type, life, life_start) → color pair
    ├── branch_bold(type, life, life_start) → bool
    │
    ├── mvaddch(b.y, b.x, ch)   ← draw directly, no erase
    │
    ├── if should_branch:
    │     tree_new_branch(LEFT, life/2)
    │     tree_new_branch(RIGHT, life/2)
    │
    └── if life <= 0:
          b.alive = false
          scatter 3-6 leaf chars in 5×3 area around death position

grow_accum -= grow_tick_ns

    ▼
if done + infinite + wait_accum >= 4s:
  erase()
  scene_plant()  ← replant resets tree pool, redraws pot

    ▼
scene_draw_static()
  draw_pot()        ← redrawn every render frame
  draw_message()    ← if show_msg
  HUD               ← top-right

    ▼
wnoutrefresh + doupdate
```

---

## Function Breakdown

### branch_char(dx, dy, type, age) → char
Purpose: pick ASCII character representing this branch segment's direction
Steps:
1. If type == BR_DYING: return `~`
2. If type == BR_TRUNK and age < 4 (young thick base):
   - dy==-1, dx==0 → `|`; dx==1 → `\`; dx==-1 → `/`; dx==0, dy==0 → `_`
3. If dy == -1 (upward): dx==0 → `|`, dx==1 → `\`, dx==-1 → `/`
4. If dy == 0 (horizontal): → `_`
5. If dy == 1 (downward/weeping): dx==1 → `\`, dx==-1 → `/`, dx==0 → `|`

---

### next_dir(type, btype, age, life, cur_dx, cur_dy) → (dx, dy)
Purpose: compute next direction for one branch step
Steps (depends on tree_type):

**BAMBOO:**
- Trunk: dy=-1 always; dx drifts 1-in-10
- Side branches: spread for first 3 steps then go horizontal

**WEEPING:**
- Trunk: dy=-1, moderate horizontal drift
- Branches: dy=-1 for 2 steps, dy=0 for 3, then dy=1 (droop)

**DWARF:**
- Trunk: mostly up, moderate horizontal drift
- Branches: wide horizontal spread

**SPARSE:**
- Trunk and branches: mostly upward, little drift

**RANDOM (default):**
- Trunk: 3-in-10 straight up, 2-in-10 up-right, 2-in-10 up-left, 1-in-10 right, 1-in-10 left
- Branches: outward bias based on BR_LEFT vs BR_RIGHT

---

### tree_step(tree, cols, rows, trunk_base_y) → bool
Purpose: advance all live branches one step; return false when all dead
Steps:
1. n_this_tick = tree.n (don't process branches added this tick)
2. For i in 0..n_this_tick-1:
   a. Skip if not alive
   b. next_dir() → new dx, dy
   c. x += dx, y += dy, age++, life--
   d. Clamp x to [0, cols-1], y to [1, rows-1]
   e. Set draw_type = BR_DYING if life <= 3, else b.type
   f. branch_char/color/bold → attr → mvaddch(y, x, ch)
   g. Branching check:
      - If trunk and dist_from_base > 3:
        - Random: 1-in-(MULT_MAX - mult + 2) chance
        - Periodic: every (mult+1) trunk steps
      - If side branch: life > 3 and shoots budget: 1-in-8 chance
   h. If should_branch: spawn LEFT + RIGHT children with life/2
   i. If life <= 0:
      - Mark dead
      - Scatter n_leaves = 3 + rand()%4 leaf chars in ±2x, ±1y area
3. Return true if any branch was alive

---

### draw_pot(cx, rows, type) → int (trunk_base_y)
Purpose: draw ASCII pot at bottom center; return row just above pot top
Steps:
1. If POT_NONE: return rows-1
2. Select art lines array (big or small pot)
3. Count lines (n), find widest line
4. top_row = rows - n
5. For each line: mvprintw(top_row+i, cx - maxw/2, line)
6. Return top_row - 1 (row above pot = where trunk starts)

---

### draw_message(msg, cols, rows)
Purpose: draw ACS bordered message box at (4, 2)
Steps:
1. Compute max_w (half screen width - 4 padding)
2. box_w = max_w + 2*MSG_PAD + 2 borders
3. Top row: ACS_ULCORNER + ACS_HLINE... + ACS_URCORNER
4. Middle row: ACS_VLINE + spaces + message + spaces + ACS_VLINE
5. Bottom row: ACS_LLCORNER + ACS_HLINE... + ACS_LRCORNER

---

### scene_plant(scene, cols, rows)
Purpose: plant a new tree (erase + reset)
Steps:
1. trunk_cx = cols / 2
2. trunk_base_y = draw_pot(trunk_cx, rows, pot_type) ← also draws pot
3. tree_reset(tree, trunk_cx, trunk_base_y, tree_type, life, mult)
4. done = false, growing = true, wait_accum = 0

---

## Pseudocode — Core Loop

```
setup:
  screen_init() — initscr, noecho, cbreak, curs_set(0), nodelay, keypad, typeahead(-1)
  color_init()
  scene_init(cols, rows) — defaults + scene_plant()
  frame_time = clock_ns()
  grow_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       erase()
       scene_plant(new cols, rows)  ← replant from scratch
       reset grow_accum, frame_time

  2. dt:
     now = clock_ns()
     dt = now − frame_time
     frame_time = now
     cap at 100ms

  3. growth ticks (fixed step at grow_fps):
     grow_tick_ns = 1_000_000_000 / scene.grow_fps
     grow_accum += dt

     if not done:
       while grow_accum >= grow_tick_ns:
         alive = scene_tick()   ← tree_step, draws to stdscr directly
         grow_accum -= grow_tick_ns
         if not alive: done = true; break
     else:
       drain grow_accum  ← keep it from accumulating while waiting

  4. infinite regrow:
     if done and (infinite or screensaver):
       wait_accum += dt
       if wait_accum >= INFINITE_WAIT_MS * NS_PER_MS:
         erase()
         scene_plant(cols, rows)

  5. FPS counter (every 500ms)

  6. frame cap: sleep to 60fps

  7. draw static + present:
     scene_draw_static(fps)   ← pot + message + HUD (no erase!)
     wnoutrefresh(stdscr)
     doupdate()

  8. input:
     ch = getch()
     q/ESC      → quit
     space      → toggle paused
     t          → cycle tree_type; erase + replant
     b          → cycle pot_type; erase + replant
     m          → toggle show_msg
     M          → multiplier++ (up to MULT_MAX); erase + replant
     N          → multiplier-- (down to MULT_MIN); erase + replant
     L          → life++ (up to LIFE_MAX); erase + replant
     K          → life-- (down to LIFE_MIN); erase + replant
     ]          → grow_fps += GROW_FPS_STEP
     [          → grow_fps -= GROW_FPS_STEP
     r          → new seed, srand(), erase + replant
```

---

## Interactions Between Modules

```
App
 ├── owns Scene (all runtime state)
 ├── owns Screen (dimensions)
 └── main loop: grow_accum → scene_tick → tree_step → mvaddch
                render frame → scene_draw_static → doupdate

Scene
 ├── owns Tree (branch pool, growth state)
 └── scene_plant() → draw_pot() + tree_reset()

Tree
 ├── tree_step() iterates branch pool
 ├── tree_step() calls next_dir() for each branch
 ├── tree_step() calls branch_char/color/bold and mvaddch directly
 └── tree_step() calls tree_new_branch() to spawn children

§7 message (draw_message)
 └── called by scene_draw_static() only

§4 pot (draw_pot)
 └── called at plant time (for trunk_base_y) and every render frame (redraw)
```

---

## Key Design: No Framebuffer

Unlike most other animations in this codebase, bonsai does **not** call `erase()` each frame during growth. The tree accumulates on screen as branches write characters one at a time. This is intentional — it creates the "watching it grow" effect.

The trade-off: you cannot partially update the tree. Once a character is drawn, it stays until the next `erase()`. The pot and HUD are redrawn every render frame to stay visible above the tree.
