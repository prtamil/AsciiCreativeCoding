/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aspect_ratio.c — Minimal demo: drawing a circle with terminal cell-aspect correction.
 *
 * CONCEPT: Terminal cells are taller than wide (typical ratio ≈ 1:2 col:row).
 * Drawing a circle with x = r·cos(θ), y = r·sin(θ) produces an ellipse on screen.
 * Fix: scale x by 2 (or equivalently scale y by 0.5) so that the rendered shape
 * appears circular to the eye.  This is the foundation of all isotropic drawing
 * in this project.
 */
#include <ncurses.h>
#include <math.h>

#define PI 3.14159265359

void draw_circle(int center_x, int center_y, int radius) {
  int x, y;
  double angle;
  for (angle = 0; angle < 2 * PI; angle += 0.1) {
    x = (int)(center_x + radius * 2 * cos(angle)); // aspect ratio of 2 for horizontal scaling
    y = (int)(center_y + radius * sin(angle));
    mvaddch(y, x, '*');
  }
}

int main() {
  initscr();              
  raw();                 
  keypad(stdscr, TRUE);  
  noecho();             
  curs_set(0);         

  int max_y, max_x;
  getmaxyx(stdscr, max_y, max_x);

  // Set the aspect ratio for the circle (fix the aspect ratio)
  int center_x = max_x / 2;
  int center_y = max_y / 2;
  int radius = 10;

  // Set up double buffering
  WINDOW *win = newwin(max_y, max_x, 0, 0);
  wbkgd(win, COLOR_PAIR(1));

  while (1) {
    werase(win);  
    draw_circle(center_x, center_y, radius); 
    wrefresh(win); 

    int ch = getch(); 
    if (ch == 'q') break; 
  }

  endwin(); 
  return 0;
}

