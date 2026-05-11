# turtle — Logo-style turtle graphics

A reference for the **turtle graphics** demos in this project. This
folder currently contains **1 self-contained C program** that draws
regular polygons by walking a pen (position + heading) vertex to vertex.
The folder is a placeholder for a planned family of turtle-driven
demos (L-systems, fractal curves, recursive trees) and is intentionally
small for now.

If you've never seen turtle graphics before, the canonical Logo
formulation is: *a pen with a position and a heading. It can do two
things — walk forward by some distance (drawing as it goes), or turn
left/right by some angle. Every closed shape, every L-system, every
fractal curve in the Logo tradition is some sequence of those two
commands.*

---

## The unifying primitive

The turtle is a 2-D state machine:

```c
typedef struct {
    float px, py;    /* pen position in pixel space */
    float heading;   /* radians, 0 = east, CCW positive */
    bool  pen_down;  /* draw a line as we move? */
    int   color;     /* ncurses colour pair index */
} Turtle;
```

Two operations:

```
turtle_forward(t, d)   :=   px += d * cos(heading);
                            py += d * sin(heading);
                            (rasterise a line from old to new pos
                             if pen_down)

turtle_turn(t, dθ)     :=   heading += dθ;
```

That's the entire abstraction. Regular n-gons fall out as
**`(forward, turn 2π/n)` repeated n times**. Squares are n=4,
triangles are n=3, dodecagons are n=12. Every Logo program is a
nesting of `forward` and `turn` calls plus the structure that drives
them (loops, recursion, lookup of L-system productions).

```
     n=5 pentagon, exterior angle 2π/5 = 72°:

                   ╱╲
                  ╱  ╲                  start at vertex 0
                 ╱    ╲                 forward L
                ╱      ╲                turn 72°    ← exterior
               ╱        ╲               forward L
              ╱          ╲              turn 72°
             ╱            ╲             ...
            ╱──────────────╲             5 times, pen returns to start
```

---

## Per-file table

| File          | What it shows                                                    | Lines |
|---------------|------------------------------------------------------------------|-------|
| `duo_poly.c`  | Two turtles draw regular polygons side-by-side (left cyan triangle, right magenta pentagon). After both close, sides increment 3→4→…→12→3, looping. Each tick walks one edge, so you watch the pen move vertex-to-vertex. | 1095  |

The file uses an **aspect-corrected pixel space** so the polygons look
visually circular rather than vertically squashed (terminal cells are
~2× taller than wide; `ASPECT = 0.5` corrects the Y coordinate). Edges
are rasterised with a DDA line fill so no cell gaps appear at any step
size, and line characters (`-`, `|`, `/`, `\`) reflect each edge's
true heading for a natural look.

---

## Building and running

```bash
gcc -std=c11 -O2 -Wall -Wextra duo_poly.c -o duo_poly -lncurses -lm
```

**Keys:**

| Key             | Action                                              |
|-----------------|-----------------------------------------------------|
| `q` / `ESC`     | quit                                                |
| `space`         | pause / resume                                      |
| `r`             | reset both turtles (keep current sides)             |
| `a` / `z`       | turtle A: +1 / -1 sides (3..12)                     |
| `s` / `x`       | turtle B: +1 / -1 sides (3..12)                     |
| `+` / `=` / `-` | draw faster / slower                                |
| `]` / `[`       | sim Hz up / down                                    |

---

## Related folders

The turtle is the Logo-style cousin of the **procedural / generational**
families:

* [`procedural/generational/`](../procedural/generational/) — recursive-
  backtracker maze, recursive tree growth, terrain generators. These
  build *structure* via recursion; turtle builds *paths* via state-
  machine walking. Same underlying idea, different output type.
* [`grids/tri_grids/`](../grids/tri_grids/) — substitution tilings
  (Sierpinski, pinwheel, Penrose). The L-system and substitution
  literature meet here: a turtle that interprets an L-system string
  is one natural way to render those tilings.
* [`raster/`](../raster/) — the GPU-style pipeline alternative. Turtle
  is the *forward* model (pen position drives output); raster is the
  *backward* model (output pixel asks "what input produced me?").

## Future direction

This folder is sized for **one file today, many tomorrow**. Planned
additions:

* `lsystem.c` — Lindenmayer-system interpreter: a turtle walking a
  rewrite-rule string. Renders Koch snowflake, Sierpinski triangle,
  dragon curve, plant-like fractals from compact `F+F−F` strings.
* `koch_curve.c` — direct recursive Koch curve without the L-system
  layer (cleaner exposition of the recursion).
* `dragon_curve.c` — Heighway dragon as a turtle walk + 90° turn pattern.
* `tree.c` — recursive branching with random length / angle perturbation.

Until those land, treat `duo_poly.c` as **the worked example of the
turtle abstraction** — pen + heading + the two operations on them — and
read it alongside the references in its CONCEPTS block (Papert's
*Mindstorms*, Wikipedia "Turtle graphics", Wikipedia "Regular polygon").
