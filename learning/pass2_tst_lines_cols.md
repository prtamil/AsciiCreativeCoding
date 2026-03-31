# Pass 2 — tst_lines_cols: Pseudocode

## Module Map
| Section | Purpose |
|---------|---------|
| main()  | Entire program: init ncurses, print size, wait for key, cleanup |

There are no separate modules, structs, or functions beyond main.

## Data Flow Diagram
```
  OS terminal size
       |
       v
  initscr()  ──────────────────────────────→  LINES, COLS (ncurses globals)
                                                    |
                                                    v
                                             printw(LINES, COLS)
                                                    |
                                                    v
                                             internal back buffer
                                                    |
                                          refresh() |
                                                    v
                                           physical terminal display
                                                    |
                                          getch()   |
                                                    v
                                           user presses key
                                                    |
                                          endwin()  |
                                                    v
                                           terminal restored, exit
```

## Function Breakdown

### main() → int
Purpose: Initialize ncurses, display terminal dimensions, wait for key, exit cleanly.

Steps:
  1. Call initscr() — enters ncurses mode, populates LINES and COLS, creates stdscr.
  2. Call printw(format, LINES, COLS) — writes formatted text into stdscr's back buffer.
  3. Call refresh() — copies back buffer to terminal (makes text visible).
  4. Call getch() — blocks until user presses a key (return value discarded).
  5. Call endwin() — restore terminal to pre-ncurses state.
  6. Return 0.

Edge cases:
  - If the terminal does not support ncurses, initscr() may fail (returns ERR or aborts).
  - LINES or COLS could be 0 in unusual environments (redirected input, no TTY).

## Pseudocode — Core Loop
```
program start:
    initialize ncurses
    read LINES (number of terminal rows) and COLS (number of terminal columns)
    write the string "this terminal has [LINES] rows and [COLS] cols" to screen buffer
    flush buffer to visible terminal
    wait for any key press
    restore terminal settings
    exit with success
```

## Interactions Between Modules
There is only one function. ncurses is the only external system. The sole interaction is:

```
main()
  calls → initscr()        [ncurses: sets up terminal, fills LINES/COLS]
  calls → printw()         [ncurses: writes to virtual screen]
  calls → refresh()        [ncurses: flushes virtual screen to terminal]
  calls → getch()          [ncurses: reads one keypress]
  calls → endwin()         [ncurses: restores terminal]
```

No shared state, no custom data structures, no signal handlers.
