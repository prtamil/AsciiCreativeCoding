/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * tst_lines_cols.c — Minimal ncurses demo: query terminal dimensions.
 *
 * DEMO: Prints "this terminal has X Rows and Y cols" using the
 *       ncurses LINES + COLS globals, then waits for any key
 *       and quits.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/tst_lines_cols.c \
 *       -o tst_lines_cols -lncurses
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : ncurses, on initscr(), queries the controlling
 *                 terminal for its dimensions (via TIOCGWINSZ ioctl
 *                 on POSIX) and stores them in two GLOBAL VARIABLES:
 *                   extern int LINES;    // row count
 *                   extern int COLS;     // column count
 *                 We just printw them and refresh.
 *
 * API contract  : LINES + COLS are READ-ONLY globals updated by
 *                 ncurses on initscr() and on KEY_RESIZE events.
 *                 They reflect the size of the FULL TERMINAL.
 *                 For a specific WINDOW (newwin), use getmaxyx().
 *
 * References    :
 *   ncurses(3X) - "VARIABLES" section.
 *   tty_ioctl(4) — TIOCGWINSZ ioctl.
 *   project ncurses_basics/framework.c — uses getmaxyx every
 *     frame in the resize handler.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * ncurses needs to know the terminal size to clip writes.  It
 * asks the kernel via the TIOCGWINSZ ioctl on initscr() and
 * stores the result in two globals.  Programs build responsive
 * layouts by reading these globals.  When the user resizes the
 * terminal, ncurses re-queries (on KEY_RESIZE) and the globals
 * update.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the kernel as a city map office.  Every terminal
 * (xterm, bash, tmux pane) is a CITIZEN with a known plot of
 * land — its rows and columns.  When ncurses moves in
 * (initscr), it asks the office "how big is my plot?" and
 * gets two numbers back.  When the user resizes the terminal,
 * the office issues a SIGWINCH ("plot dimensions changed");
 * ncurses re-queries to update its records.
 *
 * LINES vs COLS — which is which?
 * ───────────────────────────────
 *   LINES = number of ROWS (vertical extent).  "How tall."
 *   COLS  = number of COLUMNS (horizontal extent).  "How wide."
 *
 * Mnemonic: LINES counts horizontal LINES of text (top to
 * bottom).  COLS counts COLUMNS (left to right).
 *
 * Coordinate convention: (row, col) — y first, x second.
 * Most ncurses APIs follow this: mvaddch(row, col, ch),
 * mvprintw(row, col, fmt, ...), etc.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │    col=0  col=1  col=2  ...        col=COLS-1    │
 *      │  row=0  ┌──────┬──────┬──────┬───┬──────────┐    │
 *      │         │      │      │      │   │          │    │
 *      │  row=1  ├──────┼──────┼──────┼───┼──────────┤    │
 *      │         │      │      │      │   │          │    │
 *      │  row=2  ├──────┼──────┼──────┼───┼──────────┤    │
 *      │         │   the typical 80x24 has              │    │
 *      │   ...   │   COLS = 80, LINES = 24              │    │
 *      │         │                                      │    │
 *      │  LINES-1└──────────────────────────────────────┘    │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * KEY FORMULAS
 * ────────────
 *   LINES, COLS                 globals after initscr()
 *   getmaxyx(window, y, x)      same info per window
 *                               (note: macro that ASSIGNS to y, x)
 *   centre point: (LINES / 2, COLS / 2)
 *   drawing a centred X-column box of width W:
 *     x_start = (COLS - W) / 2
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • LINES + COLS are STALE if the terminal resizes without
 *     ncurses noticing.  ncurses noticed via SIGWINCH +
 *     KEY_RESIZE.  Without explicit handling, your geometry
 *     becomes wrong post-resize.  See framework.c §8 for the
 *     standard pattern.
 *   • In a tmux split or screen window, LINES + COLS reflect
 *     the PANE's size, not the host terminal's — exactly
 *     what you want.
 *   • Non-tty stdin: if your program runs with stdin redirected
 *     (e.g. `./prog < input.txt`), initscr() may behave
 *     differently.  ncurses still finds the controlling
 *     terminal in most cases.
 *   • getmaxyx is a MACRO, not a function — its arguments are
 *     ASSIGNED TO, not POINTERS.  Call as
 *     `getmaxyx(stdscr, rows, cols)` NOT
 *     `getmaxyx(stdscr, &rows, &cols)`.  Common gotcha.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Run in 80x24 default terminal: prints "80 cols, 24 rows."
 *   • Resize terminal before running: prints new dimensions.
 *   • Resize terminal AFTER initscr (this minimal program
 *     doesn't handle KEY_RESIZE): output frozen at original
 *     size.  This is why framework.c handles SIGWINCH.
 *   • Run inside a tmux pane: prints the PANE size.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that
 *      order as prose.  This is THE introduction to ncurses
 *      terminal-size handling.  Read this BEFORE
 *      ncurses_basics/framework.c, which uses these primitives
 *      throughout its resize handler.
 *   2. main() — 6 lines.  THE ENTIRE PROGRAM.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   LINES, COLS          ncurses globals (extern int).  Capitalised
 *                        because they're symbolic constants /
 *                        external state.
 *   stdscr               ncurses' default WINDOW * for the whole
 *                        terminal.
 *
 * Background you need
 * ───────────────────
 *   - C basics + #include <ncurses.h>.
 *   - printf-style format strings.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - termios, ioctl, or POSIX terminal API.  ncurses
 *     abstracts all of that.
 *   - tput, terminfo, termcap.  Same.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Four tutorials that establish ncurses terminal-size handling
 * once and for all.
 *
 *   T1  How does ncurses know the terminal size?
 *   T2  LINES + COLS globals vs getmaxyx() per-window
 *   T3  Why building responsive layouts matters
 *   T4  KEY_RESIZE — staying responsive after the user resizes
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  HOW DOES NCURSES KNOW THE TERMINAL SIZE?
 * ────────────────────────────────────────────
 * On startup (initscr), ncurses asks the kernel for the size
 * of the controlling terminal via the TIOCGWINSZ ioctl:
 *
 *     struct winsize ws;
 *     ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
 *     // ws.ws_row = number of rows
 *     // ws.ws_col = number of columns
 *
 * The kernel knows the size because terminal emulators
 * inform it on every resize:
 *
 *     terminal_emulator                 kernel
 *           │                              │
 *           │ user drags window border     │
 *           ├──────────────────────────────►
 *           │   ioctl TIOCSWINSZ           │
 *           │   (set window size)          │
 *           │                              │
 *           │                          ┌───┤
 *           │                          │   │ marks each
 *           │                          │   │ child process's
 *           │                          │   │ session as
 *           │                          └───┤ needing SIGWINCH
 *
 *     bash, ncurses, etc.    SIGWINCH      │
 *           ◄──────────────────────────────┤
 *           │   "your terminal resized!"   │
 *           │                              │
 *           │ ncurses re-queries ioctl     │
 *           ├──────────────────────────────►
 *           │   TIOCGWINSZ → new size      │
 *           ◄──────────────────────────────┤
 *           │   updates LINES, COLS        │
 *
 * ncurses presents this to your code as the LINES + COLS
 * globals (and the KEY_RESIZE pseudo-key, T4).
 *
 * T2  LINES + COLS GLOBALS vs getmaxyx() PER-WINDOW
 * ─────────────────────────────────────────────────
 * Two ways to read window dimensions:
 *
 *   A) GLOBAL VARIABLES (this file's approach):
 *
 *        printw("rows=%d cols=%d", LINES, COLS);
 *
 *      Always reflect the SIZE OF THE TERMINAL (stdscr).
 *      Updated by ncurses on initscr + after each
 *      KEY_RESIZE handling.
 *
 *   B) PER-WINDOW MACRO:
 *
 *        int rows, cols;
 *        getmaxyx(stdscr, rows, cols);
 *
 *      Reflects the size of ANY ncurses window — useful when
 *      you have multiple windows (newwin, subwin) of
 *      different sizes.
 *
 * For a single full-screen program, both are equivalent
 * since stdscr is the whole terminal.
 *
 * KEY GOTCHA: getmaxyx is a MACRO that ASSIGNS to its
 * arguments.  Don't use & address-of:
 *
 *   getmaxyx(stdscr, rows, cols);    // ✓ correct
 *   getmaxyx(stdscr, &rows, &cols);  // ✗ compile error or
 *                                    //   undefined behaviour
 *
 * (The macro expands to something like
 *  `(rows = stdscr->_maxy + 1, cols = stdscr->_maxx + 1)` —
 * it needs L-VALUES, hence no & operator.)
 *
 * Project framework.c uses getmaxyx in the main loop's
 * resize handler so the program responds to terminal
 * resizes mid-execution.  Read that file's §4 + §8 next.
 *
 * T3  WHY BUILDING RESPONSIVE LAYOUTS MATTERS
 * ───────────────────────────────────────────
 * This file just prints the dimensions and exits.  Useful as
 * a sanity check, not as a final program.  REAL programs
 * BUILD A LAYOUT FROM these dimensions:
 *
 *     int rows = LINES, cols = COLS;
 *     int hud_row = 0;                 // top
 *     int hint_row = rows - 1;         // bottom
 *     int sim_top = 1;
 *     int sim_bot = rows - 2;
 *     int cx = cols / 2, cy = rows / 2;  // centre
 *
 * Or for a centred box:
 *
 *     int box_w = 40, box_h = 12;
 *     int box_x = (cols - box_w) / 2;
 *     int box_y = (rows - box_h) / 2;
 *
 * The pattern: COMPUTE LAYOUT FROM DIMENSIONS at the start
 * of every frame (or whenever the dimensions change).
 * Never hard-code "row 23 is the bottom" — that breaks the
 * moment someone resizes.
 *
 * Project rule: every animation file derives its layout
 * from getmaxyx(stdscr, rows, cols) at the top of
 * scene_draw().  Look at any file's §6 scene to see this
 * pattern.
 *
 * T4  KEY_RESIZE — STAYING RESPONSIVE AFTER THE USER RESIZES
 * ──────────────────────────────────────────────────────────
 * This minimal file does NOT handle resize.  If the user
 * resizes the terminal AFTER `initscr` runs, the printed
 * numbers stay frozen at their initial values until the
 * program restarts.
 *
 * To handle resize, ncurses provides KEY_RESIZE: a
 * pseudo-key returned by getch when SIGWINCH was received.
 * The classic pattern:
 *
 *     while (running) {
 *       int ch = getch();
 *       if (ch == KEY_RESIZE) {
 *         // re-query dimensions; rebuild layout
 *         endwin();           // reset ncurses state
 *         refresh();          // restart with new size
 *         continue;
 *       }
 *       // ... handle other keys, draw frame, etc.
 *     }
 *
 * This gives a smooth UX: drag to resize → ncurses tells
 * your program → your code re-derives layout → next frame
 * is correctly proportioned.
 *
 * The signal-driven version (more responsive, used in
 * framework.c §8):
 *
 *     volatile sig_atomic_t g_resize = 0;
 *     void on_winch(int sig) { g_resize = 1; }
 *     signal(SIGWINCH, on_winch);
 *
 *     while (running) {
 *       if (g_resize) {
 *         g_resize = 0;
 *         endwin(); refresh();
 *       }
 *       // draw frame
 *     }
 *
 * Both work.  The signal version doesn't need to wait for
 * the next getch().
 *
 * Read framework.c next to see this pattern in production.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#include <ncurses.h>

int main(void)
{
    initscr();
    printw("this terminal has %d Rows and %d cols", LINES, COLS);
    refresh();
    getch();
    endwin();
    return 0;
}
