# Pass 1 — aspect_ratio: Draw a geometrically correct circle in the terminal

## Core Idea
Terminal cells are not square — they are roughly twice as tall as they are wide. If you naively place a dot at (cos θ, sin θ) for each angle θ, you get a tall ellipse, not a circle. This program fixes that by scaling the horizontal component by 2, producing a shape that looks round to the human eye.

## The Mental Model
Think of the terminal as graph paper where each square is actually a 1-wide by 2-tall rectangle. If you draw a circle using the standard formula, it will look squished vertically (like an oval standing upright). To compensate, you stretch all horizontal distances by 2. Now the oval is stretched back out into a circle. The program draws this corrected circle once, then enters a loop waiting for 'q' to quit.

The program uses a `while(1)` loop but the loop body is gated on `getch()` — it blocks until a key is pressed, so the animation effectively runs at "0 fps" — the circle is drawn once and then redrawn on every keypress until 'q'.

## Data Structures
No custom structs. Only three local variables in `draw_circle`:
- `x`, `y` (int): computed terminal column and row for each dot
- `angle` (double): current angle in radians, steps from 0 to 2π

## The Main Loop
Each iteration:
1. Erase the window (werase) — clear any previous frame.
2. Call `draw_circle` — recalculate and redraw all dots for the circle.
3. Refresh the window (wrefresh) — push the buffer to the screen.
4. Block on getch() — wait for a keypress.
5. If the key is 'q', break out of the loop.
6. Otherwise, loop back to step 1.

## Non-Obvious Decisions

**Why multiply the x component by 2?**
Terminal cells have an aspect ratio of approximately 2:1 (height:width). The exact ratio varies by font and terminal emulator, but 2.0 is the canonical approximation. If you draw the circle without this correction:
- A point at angle 0° (rightmost) will be `radius` columns to the right.
- A point at angle 90° (bottom) will be `radius` rows down.
- But `radius` columns is about half the physical width of `radius` rows.
- The result is a vertically tall ellipse.
Multiplying x by 2 spreads the horizontal points to compensate for the narrow cells, producing a visually round shape.

**Why use a separate WINDOW (newwin)?**
The code creates a new window the full size of the screen with `newwin`. However, `draw_circle` still calls `mvaddch` without a window argument, meaning it draws into `stdscr`, not into `win`. This is a subtle bug: `werase(win)` clears `win`, but `mvaddch` puts dots into `stdscr`. The `wrefresh(win)` then shows the empty `win` on top. In practice, the circle may still appear if ncurses overlays windows, but the window architecture here is not correctly wired up. This is a learning-stage code — the pattern to note is the intent of using a window for double buffering.

**Why `wbkgd(win, COLOR_PAIR(1))` with no `start_color()` call?**
`start_color()` is never called, so `COLOR_PAIR(1)` is undefined. `wbkgd` will set a background attribute but with no defined color pair it has no visible effect. This is harmless but incomplete — the correct pattern (as seen in `bounce_ball.c`) calls `start_color()` and `init_pair()` before using color pairs.

**Why step angle by 0.1 radians?**
At radius 10, the arc between two adjacent angle steps is `10 * 0.1 = 1` pixel (approximately). With the x-scale of 2, horizontal spacing is about 2 cells per step at the equator. Using a finer step (e.g., 0.05) would double the dot density; a coarser step would leave visible gaps.

**Why `angle < 2 * PI` (not `<=`)?**
The loop condition uses strict less-than. Since 0 and 2π map to the same point, including the final value would draw a duplicate dot at the start position. Not harmful, just redundant.

**Why `raw()` and `noecho()` and `keypad(stdscr, TRUE)`?**
- `raw()`: keys are delivered immediately without waiting for Enter.
- `noecho()`: typed characters are not echoed back to the screen.
- `keypad(stdscr, TRUE)`: function keys and arrow keys are decoded into single key codes (KEY_UP, etc.) rather than escape sequences.
These three together give you clean interactive input for a real-time program.

## State Machines
No state machine. The only "state" is whether the user has pressed 'q'. If yes, exit. If no, redraw.

```
[DRAWING] ---(key pressed, key == 'q')--> [EXIT]
   ^              |
   |__(key != 'q')_|
```

## From the Source

**Algorithm:** Draw a circle with `x = center_x + radius * 2 * cos(angle)`, `y = center_y + radius * sin(angle)`. The factor of 2 on x compensates for the ≈1:2 column:row aspect ratio of terminal cells, making the rendered shape appear circular rather than a tall ellipse.

**Rendering:** The source notes this is the foundation of all isotropic drawing in the project: scaling x by 2 (or equivalently y by 0.5) is the canonical fix applied across every simulation that draws circles or physics in terminal space.

---

## Key Constants and What Tuning Them Does
| Constant / Variable | Default | Effect of changing |
|---------------------|---------|-------------------|
| `radius` | 10 | Larger = bigger circle. Limited by terminal dimensions. |
| `2 * cos(angle)` multiplier | 2 | Change to match your terminal's actual cell aspect ratio. Most terminals are between 1.8 and 2.2. Lower = more oval, higher = wide ellipse. |
| angle step `0.1` | 0.1 rad | Smaller step = more dots, denser circle, more computation. Larger = gaps in the outline. |
| `PI` (defined as 3.14159...) | built-in | Using `M_PI` from `<math.h>` is more precise for production. |

## Open Questions for Pass 3
- Why does the code create `win` with `newwin` but draw into `stdscr` with `mvaddch`? What is the intended behavior?
- How would you query the real aspect ratio of the terminal's font instead of hardcoding 2?
- What would happen with a very large radius (larger than half the screen width)?
- How would you draw the circle continuously animated (rotating, pulsing) rather than static?
- Why does `wbkgd` need `start_color()` and `init_pair()` to be called first?

---

# Pass 2 — aspect_ratio: Pseudocode

## Module Map
| Section | Purpose |
|---------|---------|
| §1 draw_circle() | Compute and plot all dots of the aspect-ratio-corrected circle |
| §2 main()        | ncurses setup, window creation, event loop, cleanup |

## Data Flow Diagram
```
  terminal size (LINES, COLS)
       |
       v
  max_x, max_y  ──────────────────────────────────────────────────────┐
                                                                       |
  center_x = max_x / 2                                                 |
  center_y = max_y / 2                                                 |
  radius   = 10                                                        |
       |                                                               |
       v                                                               |
  draw_circle(center_x, center_y, radius)                              |
       |                                                               |
       v                                                               |
  for angle 0 → 2π (step 0.1):                                        |
    x = center_x + radius * 2 * cos(angle)   ←── aspect ratio fix ←──┘
    y = center_y + radius * sin(angle)
    mvaddch(y, x, '*')  → writes to stdscr back buffer
       |
       v
  wrefresh(win)  →  terminal screen (dots visible)
       |
       v
  getch()  →  user presses key
       |
  key == 'q'?
    yes → break → endwin → exit
    no  → loop back → werase → draw_circle → wrefresh
```

## Function Breakdown

### draw_circle(center_x, center_y, radius) → void
Purpose: Place '*' characters along a circle outline that looks geometrically round in a terminal.

Steps:
  1. Declare local integer x, y and double angle.
  2. Loop angle from 0.0 to less than 2π, incrementing by 0.1 each step.
  3. For each angle:
     a. Compute x = center_x + radius * 2 * cos(angle).
        The factor 2 compensates for the ~2:1 height:width cell aspect ratio.
     b. Compute y = center_y + radius * sin(angle).
        No correction needed vertically.
     c. Cast both to int (truncation — no rounding).
     d. Call mvaddch(y, x, '*') to write a dot into stdscr's buffer.
  4. Loop ends at 2π (one full revolution).

Edge cases:
  - No bounds checking: if x or y is outside the terminal, ncurses silently ignores out-of-bounds mvaddch calls (or may ERR silently).
  - Duplicate dots: adjacent angle steps may map to the same (x, y) cell — harmless, just redraws same cell.

### main() → int
Purpose: Set up ncurses, compute circle parameters, run the draw-wait loop, clean up.

Steps:
  1. initscr() — start ncurses.
  2. raw() — raw keyboard input (no line buffering).
  3. keypad(stdscr, TRUE) — decode special keys.
  4. noecho() — don't echo typed characters.
  5. curs_set(0) — hide cursor.
  6. getmaxyx(stdscr, max_y, max_x) — read terminal dimensions.
  7. Set center_x = max_x / 2, center_y = max_y / 2, radius = 10.
  8. Create win = newwin(max_y, max_x, 0, 0) — full-screen overlay window.
  9. wbkgd(win, COLOR_PAIR(1)) — set background color (NOTE: color_init never called, so this is a no-op effectively).
  10. Enter infinite loop:
      a. werase(win) — clear win's buffer.
      b. draw_circle(center_x, center_y, radius) — draw dots into stdscr.
      c. wrefresh(win) — refresh win to screen (NOTE: circle is in stdscr, not win).
      d. ch = getch() — block for keypress.
      e. if ch == 'q', break.
  11. endwin() — restore terminal.
  12. return 0.

Edge cases:
  - The window `win` is created and refreshed, but circle dots go into `stdscr`. This is an architectural mismatch in the code.

## Pseudocode — Core Loop

```
setup:
    start ncurses
    enable raw input, no echo, no cursor, special keys
    read terminal width and height
    set circle center to middle of screen
    set radius = 10
    create full-screen overlay window

main loop (runs forever until 'q'):
    clear overlay window buffer

    for every angle from 0 to 360 degrees (in 0.1-radian steps):
        x = center_x + radius * 2 * cosine(angle)    // stretch horizontally
        y = center_y + radius * sine(angle)
        place '*' at (row=y, col=x) in screen buffer

    flush screen buffer to terminal (show the dots)

    wait for one keypress
    if keypress is 'q':
        exit loop

teardown:
    restore terminal
    exit
```

## Interactions Between Modules
```
main()
  calls → initscr, raw, keypad, noecho, curs_set   [ncurses setup]
  calls → getmaxyx                                   [read dimensions]
  calls → newwin                                     [create window]
  calls → wbkgd                                      [set background]
  loop:
    calls → werase(win)                              [clear window]
    calls → draw_circle(cx, cy, r)
               calls → mvaddch(y, x, '*')            [write to stdscr]
    calls → wrefresh(win)                            [push win to screen]
    calls → getch()                                  [read input]
  calls → endwin()                                   [restore terminal]
```

Shared state: `center_x`, `center_y`, `radius` are computed once in main and passed to draw_circle. No global state. The window `win` is created in main but not passed to draw_circle (draw_circle uses the global stdscr implicitly).
