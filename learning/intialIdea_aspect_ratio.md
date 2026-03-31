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
