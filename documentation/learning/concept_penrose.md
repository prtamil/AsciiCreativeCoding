# Concept: Penrose Tiling

## Pass 1 — Understanding

### Core Idea
An aperiodic tiling of the plane using two tile shapes (kite and dart, or thin and thick rhombus). No translation can map the tiling to itself, yet the tiling has 5-fold rotational symmetry. Generated via de Bruijn's pentagrid duality or subdivision rules.

### Mental Model
Imagine trying to tile a floor with two differently-shaped tiles such that the pattern never repeats, yet fits together perfectly with no gaps. Penrose showed this is possible. Every finite patch of the tiling appears infinitely often — it's locally repetitive but globally aperiodic.

### Key Math
De Bruijn pentagrid method:
- Define 5 families of parallel lines with spacing 1, oriented at angles k·36° for k=0..4
- Each intersection of lines from different families maps to a vertex in the Penrose tiling
- The dual of the pentagrid is the tiling

Subdivision (inflation/deflation):
- Each tile subdivides into smaller tiles by the golden ratio φ=(1+√5)/2
- Apply recursively to generate finer tilings

### Data Structures
- Tile: {type (kite/dart or thick/thin), vertices[4]}
- Tile list
- Color: two distinct colors for the two tile types

### Non-Obvious Decisions
- **de Bruijn method is more uniform**: Subdivision generates tiles concentrated in the center. De Bruijn distributes them evenly across the plane.
- **Golden ratio everywhere**: Edge lengths are 1 and 1/φ. The 36°-72°-72° angles of the kite contain the golden ratio in their trigonometry.
- **Matching rules**: Arrows on tile edges enforce aperiodicity — tiles can only join if arrows match. Without matching rules, periodic tilings are also possible with the same shapes.
- **ASCII rendering**: Draw only tile edges using `|`, `-`, `/`, `\` characters. Fill tiles with different shading characters to distinguish types.

### Key Constants
| Name | Value | Role |
|------|-------|------|
| φ (phi) | (1+√5)/2 ≈ 1.618 | golden ratio |
| ANGLE | 36° = π/5 | base angle |
| DEPTH | subdivision depth |

### Open Questions
- Is there a relationship between Penrose tilings and quasicrystals?
- Can you show the 5-fold symmetry by rotating the view by 72°?
- Where in the tiling do you find 10-fold symmetric local patches?

---

## Pass 2 — Implementation

### Pseudocode (subdivision method)
```
# Start with a "sun" or "star" of 10 triangles
# Each triangle is half a kite or half a dart

struct Triangle { int type; complex a,b,c; }

subdivide(tri):
    if tri.type == THICK:  # acute golden gnomon
        p = tri.a + (tri.b - tri.a)/phi
        yield Triangle{THIN, tri.c, p, tri.b}
        yield Triangle{THICK, p, tri.c, tri.a}
    else:                   # obtuse golden triangle
        p = tri.b + (tri.c - tri.b)/phi
        yield Triangle{THICK, tri.a, p, tri.b}
        yield Triangle{THIN, p, tri.a, tri.c}

generate(depth):
    tris = initial_sun()
    for _ in range(depth):
        tris = flatten([subdivide(t) for t in tris])
    return tris

draw_tile(tri):
    # draw edges between vertices as lines
    draw_line(tri.a, tri.b)
    draw_line(tri.b, tri.c)
    draw_line(tri.c, tri.a)
    # fill interior with type-appropriate char
```

### Module Map
```
§1 config    — DEPTH, initial scale, center
§2 complex   — complex arithmetic (or struct with re/im)
§3 generate  — initial_sun(), subdivide(), recursion
§4 draw      — world→screen, draw_line(), fill_triangle()
§5 app       — main (generate once, draw, keys: depth, zoom, pan)
```

### Data Flow
```
initial 10 triangles → subdivide DEPTH times → tile list
→ world→screen transform → draw edges + fill interiors
```
