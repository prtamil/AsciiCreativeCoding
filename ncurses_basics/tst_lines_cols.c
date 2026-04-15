/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * tst_lines_cols.c — Minimal ncurses demo: query terminal dimensions.
 *
 * CONCEPT: After initscr(), ncurses sets the global LINES (row count) and
 * COLS (column count) to the current terminal size.  These are the primary
 * way to make layouts responsive.  getmaxyx(stdscr, rows, cols) is the
 * per-window equivalent and is safer in multi-window programs.
 */
#include <ncurses.h>

int main()
{
    initscr();
    printw("this terminal has %d Rows and %d cols",LINES,COLS);
    refresh();
    getch();
    endwin();
    return 0;
}
