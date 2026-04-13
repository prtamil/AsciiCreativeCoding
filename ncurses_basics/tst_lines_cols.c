/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
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
