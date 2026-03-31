# Pass 1 — tst_lines_cols: Print the terminal's row and column count

## Core Idea
This is the absolute minimum ncurses program. It initializes the terminal library, reads the dimensions that ncurses detected at startup, prints them, waits for any key, and exits. Its sole purpose is to verify that ncurses is working and to show how to query terminal size.

## The Mental Model
Imagine you walk into a room and immediately ask: "How big is this room?" You get the answer, read it aloud, and leave. That is this program. ncurses is the system that knows about the room (the terminal). `initscr()` opens the terminal. `LINES` and `COLS` are the two numbers it already figured out. `printw` puts them on screen. `refresh()` actually sends the output to the physical terminal. `getch()` waits so you can read the answer before the program closes. `endwin()` restores the terminal to its normal state.

## Data Structures
None. No custom types. `LINES` and `COLS` are global integer macros/variables provided by ncurses itself after `initscr()` is called.

## The Main Loop
There is no loop. The program runs exactly once:
1. Open ncurses (initscr).
2. Write a formatted string containing LINES and COLS into the virtual screen buffer.
3. Push the buffer to the real terminal (refresh).
4. Block until the user presses any key (getch).
5. Restore terminal (endwin).
6. Exit.

## Non-Obvious Decisions

**Why call refresh() before getch()?**
`printw` writes to ncurses' internal back buffer, not directly to the terminal. Nothing appears on screen until `refresh()` is called. If you omit `refresh()`, the user sees a blank screen and then the terminal closes. Always: write → refresh → optionally wait.

**Why endwin() at the end?**
ncurses puts the terminal into a special raw mode (cursor hiding, no echo, etc.). `endwin()` reverses all of that. Without it, the terminal is left in an unusable state after the program closes.

**Why LINES and COLS (all caps)?**
These are ncurses global variables (or macros wrapping them) that are set when `initscr()` runs. They reflect what the OS reported about the terminal window size. They are not updated live — if the terminal is resized while running, LINES/COLS do not change until you call `refresh()` or handle SIGWINCH.

## State Machines
Not applicable. This program has a single linear execution path — no loops, no states.

## Key Constants and What Tuning Them Does
No tunable constants. The program reads whatever the terminal reports.

## Open Questions for Pass 3
- What happens if the terminal is resized while waiting on getch()? Does LINES/COLS update?
- What is the minimum ncurses setup to make LINES/COLS reliable before calling initscr() returns?
- How would you query terminal size without ncurses (using ioctl TIOCGWINSZ)?
- What is the difference between LINES/COLS and getmaxyx(stdscr, y, x)?

---

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
