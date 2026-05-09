/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aspect_ratio.c — Minimal demo: drawing a circle with terminal
 *                  cell-aspect correction.
 *
 * DEMO: A static circle drawn at the centre of the screen with the
 *       horizontal coordinate scaled by 2× to compensate for the
 *       2:1 height-to-width ratio of typical monospace terminal
 *       cells.  Press q to quit.
 *
 * Section map:
 *   §1 helpers — draw_circle (the entire lesson)
 *   §2 main    — ncurses init + draw loop
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/aspect_ratio.c \
 *       -o aspect_ratio -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Sample a circle parametrically:
 *                   x = cx + r · cos(θ),  y = cy + r · sin(θ)
 *                 for θ ∈ [0, 2π) at small angular steps.  Stamp '*'
 *                 at each (x, y) cell.  Without correction, the result
 *                 is a vertical ellipse (taller than wide) because
 *                 terminal cells are NOT square.  Fix: multiply the
 *                 horizontal component by ASPECT (typically 2) to
 *                 stretch the figure horizontally back into a circle.
 *
 * Math          : Terminal cells have approximate dimensions:
 *                   cell_width  ≈ 8  pixels (font advance)
 *                   cell_height ≈ 16 pixels (font line height)
 *                 So a "1 unit horizontal" cell movement covers
 *                 HALF as many pixels as a "1 unit vertical" cell
 *                 movement.  To draw isotropic geometry, you must
 *                 either:
 *                   (a) multiply x-displacement by  cell_height/cell_width = 2
 *                   (b) multiply y-displacement by  cell_width/cell_height = 0.5
 *                 (a) makes the figure WIDER (used here for a circle
 *                 that stays at a useful screen size).
 *                 (b) makes the figure SHORTER (used by raster/ files
 *                 where the world coords are pre-fixed).
 *
 * Performance   : O(circumference) glyph stamps per frame.  At
 *                 angular step 0.1 rad, ~63 glyphs per circle.
 *                 Trivial.
 *
 * References    :
 *   project artistic/hindu_mandalas.c §4 polar — same ASPECT trick
 *     applied to polar→cell mapping for mandalas.
 *   project ncurses_basics/framework.c §4 coords — the canonical
 *     pixel↔cell bridge.
 *   ncurses(3X) man page.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Terminal cells aren't square.  Geometric formulas produce
 * BIASED shapes if you don't compensate.  Fix the bias in ONE
 * PLACE — at the moment you convert from "world coordinates"
 * to "cell coordinates."  Every other layer (the math, the
 * physics, the user interface) lives in isotropic world
 * coordinates and ignores the screen's quirk.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine plotting on graph paper where each square is 1cm
 * wide and 2cm tall.  A circle of radius 5 plotted by
 * x = r cos θ, y = r sin θ will visually look TALLER THAN
 * WIDE — its width spans 5 squares (5cm horizontal) while its
 * height spans 5 squares (10cm vertical, since each square is
 * 2cm tall).  To get a CIRCULAR figure on this paper, you
 * STRETCH THE X COORDINATE by 2 before plotting.  Or you
 * SHRINK Y by 0.5.  Either works; both restore isotropy.
 *
 * Terminal cells are exactly this graph-paper situation, just
 * approximate (the 2:1 ratio depends on font).
 *
 * THE FIX
 * ───────
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   no correction:           with x-stretch ×2:    │
 *      │                                                  │
 *      │     . . .                       . . . . . .      │
 *      │   .       .                  .             .     │
 *      │  .         .              .                 .    │
 *      │  .         .              .                 .    │
 *      │   .       .                  .             .     │
 *      │     . . .                       . . . . . .      │
 *      │                                                  │
 *      │   tall ellipse              circular              │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. For θ ∈ [0, 2π) in steps of 0.1:
 *  2.   x = cx + r · 2 · cos θ      ← × ASPECT correction here
 *  3.   y = cy + r     · sin θ
 *  4.   stamp '*' at (round(y), round(x))
 *
 * Two cents of arithmetic.  One multiplier.  Universal trick.
 *
 * KEY FORMULA
 * ───────────
 *   ASPECT = cell_height / cell_width ≈ 2  (for typical
 *                                           monospace fonts)
 *   Apply ASPECT to ONE axis only, NOT both.  Two ways:
 *
 *     (a) WIDEN x:  x_cell = cx + r · ASPECT · cos θ
 *                   y_cell = cy + r          · sin θ
 *
 *     (b) NARROW y: x_cell = cx + r · cos θ
 *                   y_cell = cy + r · sin θ / ASPECT
 *
 * (a) and (b) produce IDENTICAL shapes (just scaled by the
 * full ASPECT factor).  Choose by which axis you'd rather
 * have at the natural scale.  This file uses (a) — radius
 * spans natural cells along the y-axis.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Different fonts give slightly different ASPECT ratios.
 *     Most monospace fonts target 2.0 ± 10%.  Tune to taste.
 *   • Some terminals ignore the font's metrics and use
 *     non-standard cell dimensions (e.g. the macOS Terminal
 *     "Pro" theme).  Empirically tune ASPECT.
 *   • Step size: 0.1 rad ≈ 6° per sample.  At radius 10 cells
 *     that's 0.5 cells per step — fine.  At radius 100 cells
 *     it's 5 cells per step → visible gaps.  Reduce step
 *     size proportionally to radius for large circles.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Default radius 10: figure looks circular.  Measure with
 *     a ruler against the screen if sceptical.
 *   • Disable the · 2 multiplier (replace with · 1): figure
 *     becomes a clearly vertical ellipse.
 *   • Replace · 2 with · 1 on x AND replace · 1 with · 0.5
 *     on y: figure shrinks to half-width / quarter-area but
 *     stays circular.  ASPECT applies to the RATIO, not the
 *     absolute size.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that
 *      order as prose.  This is the CANONICAL ASPECT-RATIO
 *      LESSON — every file that does geometric drawing in this
 *      project applies the same trick.  Read this first if
 *      "why does my circle look squashed?" is on your mind.
 *   2. §1 helpers / draw_circle — 9 lines.  THE LESSON.
 *   3. §2 main — minimal ncurses init + draw + getch.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   center_x, center_y      circle centre in CELL COORDINATES.
 *   radius                  in CELL UNITS along the y-axis.
 *   ASPECT (here: literal 2)  cell_height / cell_width ratio.
 *
 * Background you need
 * ───────────────────
 *   - Trig: (cos θ, sin θ) is a unit vector at angle θ.
 *   - Basic ncurses: initscr, mvaddch, getch.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Anti-aliasing.  Cell stamps; no smoothing.
 *   - Bresenham circle algorithm.  Parametric sampling is
 *     simpler, fine at terminal resolution.
 *   - Font metrics APIs.  Empirical ASPECT = 2 covers most
 *     terminals.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Four tutorials that establish the aspect-ratio lesson once
 * and for all.
 *
 *   T1  Why a circle becomes an egg
 *   T2  Two ways to fix it (and why one place, not many)
 *   T3  Tuning ASPECT for your specific terminal
 *   T4  Where this trick reappears across the project
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHY A CIRCLE BECOMES AN EGG
 * ───────────────────────────────
 * Standard parametric circle:
 *
 *     for θ ∈ [0, 2π):
 *       x = cx + r · cos θ
 *       y = cy + r · sin θ
 *       paint cell (x, y)
 *
 * In a graphics framework with SQUARE PIXELS, this gives a
 * perfect circle.  In a terminal with 2:1 cells, it gives an
 * EGG STANDING ON ITS POINT — the y direction takes twice
 * the screen real estate per unit as the x direction.
 *
 * Concretely: at r=10:
 *   - At θ=0:    x=cx+10, y=cy   → 10 cells right
 *   - At θ=π/2:  x=cx,    y=cy+10 → 10 cells DOWN, but each
 *                                   cell is 16px tall = 160px
 *
 * Horizontal extent of the figure: ~80px (10 cells × 8px).
 * Vertical extent: 160px (10 cells × 16px).  Ratio 1:2 —
 * tall ellipse.
 *
 * T2  TWO WAYS TO FIX IT (AND WHY ONE PLACE, NOT MANY)
 * ────────────────────────────────────────────────────
 * Two equivalent corrections:
 *
 *   (A) STRETCH X: multiply x-displacement by ASPECT (≈ 2)
 *       x = cx + r · ASPECT · cos θ
 *       y = cy + r          · sin θ
 *
 *       Effect: figure is twice as WIDE in cells as in
 *       y-cells, but PIXELS are now equal in both axes.
 *
 *   (B) SHRINK Y: divide y-displacement by ASPECT
 *       x = cx + r · cos θ
 *       y = cy + r · sin θ / ASPECT
 *
 *       Effect: figure is half as TALL in cells; pixels
 *       again equal.
 *
 * Both produce a visually circular figure.  Choose (A) when
 * you want the figure to LOOK BIG (2× cell width = 2× screen
 * presence); choose (B) when working in a coordinate system
 * where the y-axis natively has more cells to spare.
 *
 * THE PROJECT'S PROJECT-WIDE RULE: apply the correction in
 * ONE PLACE — typically inside a `polar_to_cell()` or
 * `pixel_to_cell()` helper in §4 coords.  Every other layer
 * of code works in ISOTROPIC WORLD COORDINATES and is
 * unaware of the cell aspect.  This way:
 *
 *   - Bugs from forgetting the correction can't happen
 *     mid-program (the helper always applies it).
 *   - Tuning ASPECT changes one constant, fixes the whole
 *     program.
 *   - Math everywhere reads naturally — sin and cos look
 *     right.
 *
 * In artistic/hindu_mandalas.c, polar_to_cell does this
 * encapsulation for radial designs.  In raster/ files, the
 * NDC-to-screen mapping does it for 3-D scenes.
 *
 * T3  TUNING ASPECT FOR YOUR SPECIFIC TERMINAL
 * ────────────────────────────────────────────
 * ASPECT = 2 is a good default.  Real terminal cell aspects
 * (height/width) vary:
 *
 *   xterm default       2.0
 *   gnome-terminal      ~2.0
 *   iTerm2 default      ~2.0
 *   Windows Terminal    ~2.05
 *   "Tall" themes       ~2.2
 *   "Wide" themes       ~1.8
 *   urxvt default       2.0
 *
 * The formula is:
 *
 *     ASPECT = font_advance_height / font_advance_width
 *
 * which depends on the FONT (not the terminal emulator).
 * To measure precisely:
 *
 *   1. Draw a "pure" parametric circle at radius 10 with
 *      ASPECT = 1 (no correction).
 *   2. Measure (with a ruler or screenshot pixel measurement)
 *      width and height of the resulting figure in PIXELS.
 *   3. ASPECT = height / width.
 *   4. Hard-code that ASPECT in your file.
 *
 * For most users, ASPECT = 2 is close enough.  Off by 10%?
 * The figure looks like an ellipse with axis ratio 1.1 —
 * almost circular.  Acceptable.
 *
 * T4  WHERE THIS TRICK REAPPEARS ACROSS THE PROJECT
 * ─────────────────────────────────────────────────
 * The same ASPECT correction appears in DOZENS of files in
 * this project.  Find them all by searching for "ASPECT" or
 * "0.5" in cell-rendering code.  Highlights:
 *
 *   - artistic/hindu_mandalas.c §4 polar_to_cell:
 *       row = cy + r · sin θ · 0.5     ← (B) shrink-y form
 *
 *   - artistic/islamic_mandalas.c §4 polar:
 *       same ASPECT 0.5 trick
 *
 *   - matrix_rain/pulsar_rain.c §1.2:
 *       ASPECT = 0.45 for slightly tall fonts
 *
 *   - matrix_rain/sun_rain.c §5.2 ray_init:
 *       sin_a baked with ASPECT once at init for performance
 *
 *   - turtle/duo_poly.c §4 coords + draw_polygon:
 *       y = cy + r · sin θ · ASPECT
 *
 *   - algorithms/visibility_polygon.c §4 geometry:
 *       y_geo = y_cell · ASPECT_Y    ← isotropic geo space
 *
 *   - any pixel-space simulation (animation/, flocking/,
 *     physics/): §4 coords has CELL_W=8, CELL_H=16, and the
 *     px_to_cell_x/y helpers apply the implicit aspect.
 *
 * Once you understand THIS file, every other file's aspect
 * handling reads as the same lesson.  The implementations
 * differ slightly (when to pre-bake ASPECT into a constant,
 * etc.) but the principle is identical.
 *
 * One trick.  One concept.  Used everywhere.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#include <ncurses.h>
#include <math.h>

#define PI 3.14159265359

/* ===================================================================== */
/* §1  helpers                                                            */
/* ===================================================================== */

/* draw_circle — parametric circle sampling with ASPECT correction.
 *
 * The "* 2" on the cos line is the WHOLE LESSON.  Without it, this
 * draws a vertical ellipse; with it, a circle.  See T1 above. */
void draw_circle(int center_x, int center_y, int radius) {
  int x, y;
  double angle;
  for (angle = 0; angle < 2 * PI; angle += 0.1) {
    x = (int)(center_x + radius * 2 * cos(angle)); /* ASPECT = 2 (T2 form A) */
    y = (int)(center_y + radius * sin(angle));
    mvaddch(y, x, '*');
  }
}

/* ===================================================================== */
/* §2  main                                                               */
/* ===================================================================== */

int main() {
  initscr();
  raw();
  keypad(stdscr, TRUE);
  noecho();
  curs_set(0);

  int max_y, max_x;
  getmaxyx(stdscr, max_y, max_x);

  int center_x = max_x / 2;
  int center_y = max_y / 2;
  int radius = 10;

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
