# Pass 1 — wireframe.c: Vertex-edge wireframe 3D shapes with Bresenham line drawing

## Core Idea

wireframe.c draws four different 3D shapes (cube, sphere, pyramid, torus) as
wireframes — the skeleton of edges connecting vertices — directly into the
terminal using ASCII characters. It is not a raymarcher and not a rasteriser
of filled surfaces. Each shape is defined as a list of 3D vertices and a list
of pairs of vertex indices (edges). Each frame, every vertex is rotated and
perspective-projected to a 2D screen coordinate, then each edge is drawn as an
ASCII line using Bresenham's algorithm.

The key visual technique is choosing different characters for line segments
depending on their slope: `-` for horizontal, `|` for vertical, `/` and `\`
for diagonals. Curved shapes (sphere, torus) use `o` for all segments because
their constantly-changing arc slopes would produce visual noise if slope
characters were used.

## The Mental Model

Think of building a wire model of each shape with straight wire segments, then
holding it in front of a lamp and looking at its shadow on a flat screen. Each
frame you rotate the wire model slightly and re-project the shadow.

The process:
1. Every shape is a lookup table of 3D points (vertices) and connections
   between them (edges).
2. Each frame, all vertices are rotated around Y then around X by the current
   rotation angles.
3. Each rotated vertex is projected to 2D using perspective division.
4. Each edge is then two projected 2D points, and a line of characters is
   drawn between them using Bresenham's integer line algorithm.
5. The terminal is cleared each frame and the new lines are drawn.

## Data Structures

### `Vec3`
A 3D floating-point vector: `{x, y, z}`. All geometry lives in Vec3 value types —
no pointers or heap allocation for math. Passed and returned by value (the
compiler optimises this to registers).

### `Edge`
A pair of vertex indices `{a, b}`. An edge means "draw a line from vertex `a`
to vertex `b`".

### `Shape`
The full definition of one drawable shape:
- `name` — string name for HUD display.
- `nv` — number of vertices.
- `ne` — number of edges.
- `verts[MAX_VERTS]` — up to 400 world-space 3D vertex positions. For all
  shapes, vertices are scaled so the maximum radius is approximately 1. This
  normalisation is critical because `fov_from_screen()` assumes unit radius.
- `edges[MAX_EDGES]` — up to 800 edge index pairs.
- `color` — which ncurses color pair to use for this shape.
- `curved` — flag: if true, draw `o` for all edge pixels; if false, draw slope
  characters (`-`, `|`, `/`, `\`).

### `P2`
A projected 2D point: `{col, row, z}`. Includes the original world-space z so
the caller can skip edges that pass behind the camera.

### `Canvas`
A heap-allocated `rows * cols` grid of `char` and `ShapeColor`. One `ch` and
one `col` per terminal cell. Only non-zero `ch` cells are drawn. Allocated fresh
on startup and reallocated on terminal resize.

### `Scene`
All animation state:
- `shapes[4]` — the four built-in shapes (pre-generated).
- `canvas` — the current framebuffer.
- `active` — index of which shape is displayed.
- `rx`, `ry` — current accumulated rotation angles (radians).
- `rot_x`, `rot_y` — rotation speeds (radians/second).
- `zoom` — scale multiplier on the field of view.
- `paused` — flag.

## The Main Loop

Each iteration:
1. Resize check — rebuild Canvas if terminal changed size.
2. Delta time — nanoseconds since last frame, capped at 100ms.
3. Simulation accumulator — call `scene_tick()` for each elapsed tick.
   `scene_tick()` adds `rot_x * dt` to `rx` and `rot_y * dt` to `ry`.
4. `scene_render()` — the core pipeline:
   a. `canvas_clear()` — zero all cells.
   b. Compute `fov` (field of view in terminal rows) from screen size and zoom.
   c. Project all vertices of the active shape.
   d. For each edge, draw a Bresenham line on the canvas.
5. FPS counter update.
6. Frame cap sleep.
7. `screen_draw()` — erase ncurses screen, call `canvas_draw()`, draw HUD.
8. `screen_present()`.
9. Input handling.

### Inside `scene_render()` — vertex projection

For each vertex `v` in the active shape:
1. `rot_yx(v, rx, ry)` — rotate around Y by `ry`, then around X by `rx`.
2. `project_to_screen(rotated_v, fov, ox, oy)` — perspective divide:
   - `scale = fov / (CAM_DIST - v.z)`. The camera is at z=+5, shapes at origin.
   - `col = ox + v.x * scale`.
   - `row = oy - v.y * scale / CELL_AR`. Dividing by CELL_AR (2.0) corrects
     the aspect ratio.
   - Returns `P2 {col, row, z}`.

### Inside `canvas_line()` — Bresenham

Standard Bresenham integer line algorithm handling all 8 octants. For each
pixel on the line it calls `canvas_set()`. The character placed at each pixel
is determined once before the loop: for straight-edge shapes, analyse the
slope of the overall edge to pick one of `-`, `|`, `/`, `\`; for curved shapes,
always `o`.

## Non-Obvious Decisions

### Why unit-radius vertices?

All four shapes are built with vertices whose maximum distance from the origin
is approximately 1 (cube has half-extents of 1, sphere has radius 1, etc.).
This is essential because `fov_from_screen()` computes the field of view to
make a unit-radius shape fill `FILL * screen_half_height` rows. If shapes had
different inherent sizes the fov formula would need shape-specific scaling.

### Why fov in "terminal rows"?

The field of view is stored as the number of terminal rows that correspond to
one world-space unit at the focal plane. This is more intuitive for a
terminal renderer than an angle. The formula:

    fov_rows = FILL * (rows/2) * CAM_DIST

says: project the unit-radius sphere onto a screen at camera distance CAM_DIST,
and it should span FILL * (rows/2) rows. This lets the shape always fill the
desired fraction of the screen.

The column dimension is also considered:
    fov_cols = FILL * (cols/2) * CAM_DIST / CELL_AR

and the minimum of the two is used, so the shape fits in both dimensions.

### Why CELL_AR = 2.0 and where it appears

Terminal cells are roughly twice as tall as wide. This means the same world-space
distance spans twice as many physical pixels vertically as horizontally. Without
correction, a circle would appear as a tall ellipse.

The correction is applied in `project_to_screen()` by dividing the projected
row by CELL_AR. This compresses the vertical extent so a unit sphere appears
round.

It also appears in `fov_from_screen()` to account for the fact that columns and
rows represent different physical distances.

### Why Bresenham and not floating-point lines?

Bresenham uses only integer arithmetic and always produces a connected path
through discrete grid cells with no gaps. A naive floating-point approach would
have sub-pixel gaps at the transition between diagonal and axis-aligned segments.
For a terminal renderer where each cell is a big "pixel", Bresenham is the
correct choice.

### Slope characters vs uniform dot

For flat-faced shapes (cube, pyramid), using `-`, `|`, `/`, `\` makes the edges
look like clean geometric lines — your eye reads them as actual straight edges.
For curved shapes (sphere arcs, torus rings), each short segment is an
arc of a great circle and its slope changes constantly. If slope characters were
used, adjacent pixels of the same arc would sometimes differ between `/` and `-`
and look like visual noise. The uniform `o` is visually coherent for curves.

### Why Tab resets rx/ry to 0.4 / 0.6?

When switching shapes, the rotation state is reset to a specific initial angle
rather than zero. This ensures the new shape appears at a visually interesting
orientation (slightly rotated, not face-on to the camera) immediately after the
switch, without a disorienting jump if the previous shape had accumulated large
angles.

## State Machines

```
Shape cycling:
  [cube] --Tab--> [sphere] --Tab--> [pyramid] --Tab--> [torus] --Tab--> [cube]
                                                        (wraps via modulo)
  Transition: resets rx=0.4, ry=0.6.

Pause:
  [RUNNING] --space--> [PAUSED]
  [PAUSED]  --space--> [RUNNING]

Signals:
  [ALIVE] --SIGINT/SIGTERM--> running=0 --> [EXIT]
  [ALIVE] --SIGWINCH--> need_resize=1 --> canvas rebuilt next frame
```

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|---|---|---|
| `CAM_DIST` | 5.0 | Distance from camera to origin. Smaller = more fisheye distortion. Must stay > 1 (shape radius) |
| `FILL` | 0.82 | Fraction of screen the shape fills. 1.0 would touch the edge; 0.5 would be half-size |
| `CELL_AR` | 2.0 | Physical cell height/width ratio. Change if your terminal has a non-standard font |
| `ROT_X_DEF` | 0.50 | Initial vertical tumble speed in radians/sec |
| `ROT_Y_DEF` | 0.85 | Initial horizontal spin speed. Higher than X for natural-looking motion |
| `SPHERE_STACKS` | 6 | More stacks = more visible latitude rings on the sphere. Adds to vertex/edge count |
| `SPHERE_SLICES` | 8 | More slices = more longitude lines. Too many and they overlap at terminal resolution |
| `TORUS_MAJOR` | 12 | Number of ring circles on the torus. More = smoother ring |
| `TORUS_MINOR` | 6 | Number of tube circles per ring position. More = smoother tube |

## Open Questions for Pass 3

- The shapes are built once at startup and never changed. If you wanted to
  smoothly morph between shapes, what data structure change would enable that?
- Bresenham's algorithm as implemented here always picks one character per
  pixel based on the overall edge slope, not the local slope. For a long
  diagonal edge, the local slope at each pixel is the same as the overall slope,
  so this is fine. But for a curved edge drawn as many short segments, each
  segment picks its own character. Would it look better to pick the character
  per pixel based on local Bresenham direction?
- There is no z-buffer. Behind-camera vertices are clipped by checking `p.z < -CAM_DIST + 0.1`. But for a front-to-back or back-to-front consistent render,
  what would you need to add?
- The sphere and torus use `curved=true` and draw `o`. Is there a principled
  way to choose the correct slope character for each `o` pixel based on the
  arc tangent direction at that pixel?
- What would happen if CAM_DIST were smaller than the maximum vertex radius
  (1.0)? Vertices behind the camera would have negative `denom` in `project_to_screen`. The current code returns a sentinel value `z = -9999`. How would you robustly clip edges that straddle the camera plane?
