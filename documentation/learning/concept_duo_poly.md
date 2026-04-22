# Concept: Duo Poly — Dual Turtle Polygon Animator

## Pass 1 — Understanding

### Core Idea
Two turtles simultaneously draw regular polygons step-by-step in the terminal.
Each turtle holds a pen and walks around the vertices of its polygon one edge per tick —
you watch both polygons build from nothing to completion in real time.
After both close, the turtles auto-reset with one extra side each (3 → 4 → … → 12 → 3).

### Mental Model
A regular polygon inscribed in a circle: N equally-spaced points on the circumference,
connected in order. The turtle starts at vertex 0, draws a line to vertex 1, then to vertex 2,
and so on until it returns to vertex 0 to close the shape. The key insight is that
"turtle moving" and "polygon vertex sequence" are the same thing — there is no steering,
just a precomputed vertex list driven by one formula.

Two turtles run in parallel. Left turtle (cyan) draws polygon A; right turtle (magenta)
draws polygon B. Both share the same tick clock so they advance together.

### Key Equations

**Vertex positions** (regular n-gon inscribed in circle of radius R, centered at cx, cy):
```
θ_k = θ₀ + k · (2π / n)    for k = 0, 1, …, n
vertex_x[k] = cx + R · cos(θ_k)
vertex_y[k] = cy + R · sin(θ_k) · ASPECT
```
`ASPECT = CELL_W / CELL_H = 8/16 = 0.5` corrects for non-square terminal cells.
Without it, vertical distances appear twice as large as horizontal — polygons look squished.

**Auto-close**: vertex n = vertex 0 because cos/sin are 2π-periodic. No special case needed.

**Polygon fitting** (choosing radius R from terminal size):
```
max_r_x = cols × 0.21          (fits in one half, leaves margin near divider)
max_r_y = (rows − 4) × 0.40 / ASPECT    (fits vertically, avoids HUD rows)
R = min(max_r_x, max_r_y) × 0.85
```

**Edge timing** (fixed-step accumulator):
```
edge_timer -= dt    (each sim tick)
when edge_timer ≤ 0:
    edge++          (advance pen to next vertex)
    edge_timer += 1 / eps    (eps = edges per second)
```

### Data Structures
- `Turtle` struct: `cx`, `cy`, `radius`, `sides`, `edge` (0 = not started, sides = done),
  `edge_timer`, `eps`, `start_angle`, `cpair`, `done`
- No canvas or trail buffer — `turtle_draw()` replays all completed edges from vertex 0
  every frame (O(n) per frame, n ≤ 12, trivial cost)
- Two `Turtle` structs inside `Scene`

### Non-Obvious Decisions
- **Replay-based draw**: Instead of storing drawn cells, `scene_draw` recomputes all
  edges from scratch each frame. For small polygons (≤ 12 sides) this is cheaper than
  maintaining a canvas and ensures a clean redraw after resize.
- **ASPECT on Y only**: X positions are in columns directly; Y positions are multiplied by
  ASPECT = 0.5 so the shape has equal physical pixel extent in both axes. This is the one
  place where coordinate correction lives — the rest of the code sees only cell coordinates.
- **DDA segment fill**: `put_seg()` steps along the longer axis to fill all cells between
  two vertices. Without this, short-step segments leave gaps at large radii.
- **`angle_char()`**: Chooses `-` `|` `/` `\` based on `atan2(dy, dx) mod π` so each edge
  looks like a line, not just a dot at each cell.
- **Timer starts at `1/eps`**: The first edge fires after one full interval, not immediately.
  This gives the user a moment to see the starting vertex before drawing begins.
- **Resize re-init**: On SIGWINCH, both turtles are re-initialised with current side counts
  so the radius recomputation fits the new terminal size.

### Open Questions
- Can the turtle draw a star polygon (n/k notation — skip every k-th vertex)?
- What if both turtles share the same center and different starting angles — do they interfere
  visually in an interesting way?
- Could a third turtle trace the Minkowski sum of the two polygons?
- What happens if `eps` is fractional — can you show partial edge drawing with alpha lerp?

---

## Pass 2 — Implementation

### Pseudocode
```
poly_vertex(t, i):
    a = t.start_angle + i * 2π / t.sides
    return (t.cx + t.radius * cos(a),
            t.cy + t.radius * sin(a) * ASPECT)

turtle_tick(t, dt):
    if t.done: return
    t.edge_timer -= dt
    while t.edge_timer <= 0:
        t.edge++
        if t.edge >= t.sides:
            t.done = true; break
        t.edge_timer += 1 / t.eps

turtle_draw(t, w, cols, rows):
    for e in 0..t.edge-1:
        (x0, y0) = poly_vertex(t, e)
        (x1, y1) = poly_vertex(t, e+1)
        put_seg(w, x0, y0, x1, y1, color_attr(t.cpair), cols, rows)
    if not t.done:
        (hx, hy) = poly_vertex(t, t.edge)
        draw '@' at (round(hx), round(hy))

scene_tick(s, dt, cols, rows):
    turtle_tick(s.tA, dt)
    turtle_tick(s.tB, dt)
    if s.tA.done and s.tB.done:
        s.reset_timer += dt
        if s.reset_timer >= RESET_DELAY:
            na = s.tA.sides + 1 (or wrap to SIDES_MIN)
            nb = s.tB.sides + 1
            turtle_init(s.tA, ..., na, ...)
            turtle_init(s.tB, ..., nb, ...)
            s.reset_timer = 0
```

### Module Map
```
§1 config        — EPS_DEFAULT, SIDES_MIN/MAX, RESET_DELAY, SIM_FPS constants
§2 clock         — clock_ns(), clock_sleep_ns()
§3 color         — 7 pairs: cyan(A) / magenta(B) / yellow(HUD) / green / red / blue / white
§4 coords        — CELL_W=8, CELL_H=16, ASPECT=0.5; angle_char() for segment glyphs
§5 entity        — Turtle struct; turtle_init(); turtle_tick(); poly_vertex(); put_seg(); turtle_draw()
§6 scene         — Scene{tA, tB, eps, reset_timer, paused}; scene_init(); scene_tick(); scene_draw()
§7 screen        — screen_init/draw/present/resize; HUD + hint bar
§8 app           — App global; signals; app_handle_key(); main loop (dt → accum → tick → draw)
```

### Data Flow
```
scene_init(cols, rows)
  → turtle_init × 2   (compute cx/cy/radius from terminal dims, set sides/eps)

each frame:
  sim_accum += dt
  while sim_accum >= tick_ns:
    scene_tick(dt_sec, cols, rows)
      → turtle_tick(tA, dt)   (decrement timer, advance edge when expired)
      → turtle_tick(tB, dt)
      → if both done: reset_timer++; auto-reset at RESET_DELAY
    sim_accum -= tick_ns

scene_draw(alpha, dt_sec)
  → draw divider '|' at cols/2
  → turtle_draw(tA)   (replay edges 0..edge-1 + head @)
  → turtle_draw(tB)
  → polygon name labels (Triangle/Square/Pentagon/…)
  → DONE banner if both complete

screen_draw → HUD (fps / sim Hz / eps / pause state)
            → hint bar (key guide)
```

### Key Loop
```
turtle_draw replays all edges each frame from poly_vertex().
No canvas needed — polygon has at most SIDES_MAX=12 edges.
Each edge: DDA from (x0,y0) to (x1,y1), character chosen by atan2(dy,dx).
Turtle head '@' drawn after edges to appear on top.
```

## From the Source

**Algorithm:** Regular polygon via turtle graphics. A regular n-gon inscribed in a circle
of radius R has vertices at angles θ_k = θ₀ + k·(2π/n). The turtle starts at vertex 0 and
walks to vertex 1, 2, … n (= vertex 0 again) in sequence. Each edge is rasterised with a DDA
line fill so no cell gaps appear at any step size.

**Aspect fix:** Terminal cells are ~2× taller than wide (CELL_H/CELL_W = 2). The polygon Y
coordinates are scaled by ASPECT = CELL_W/CELL_H = 0.5 so the shape is visually round/square
rather than vertically compressed. Both turtles use the same correction consistently in
`poly_vertex()`.

**Timing:** Fixed-step accumulator (framework.c §8). Each tick decrements a per-turtle edge
timer; when it expires, the edge index advances and the timer resets to `1/eps`. Speed is
controlled by `eps` (edges per second), adjustable via `+/-`. Default EPS_DEFAULT = 1.5 edges/s.

**Auto-cycle:** After both polygons complete, `reset_timer` accumulates. At `RESET_DELAY = 2.0s`
both turtles reinitialise with `sides + 1` each, cycling 3 → 4 → … → 12 → 3 indefinitely.

**References:** Turtle graphics — Logo language, Seymour Papert (1967); extended in L-system
rendering. This program is the pure-polygon foundation before string-rewriting in l_system.c.
