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
