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
