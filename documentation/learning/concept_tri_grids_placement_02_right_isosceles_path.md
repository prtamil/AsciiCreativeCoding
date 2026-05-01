# Concept: Right-Isosceles Line-of-Sight Path

## Pass 1 — Understanding

### Core Idea
Same line-walk algorithm as `01_equilateral_path`, on the axis-aligned half-rect lattice. Pin START and END at two triangles; the path is the ordered list of UR/LL triangles a straight pixel line passes through. Computed by sampling the line at `step = size · 0.25` and asking `pixel_to_tri` for the wedge.

### Walk
Same as 01_equilateral_path; the only difference is the pixel→lattice formula:
```
a = px / size,  b = py / size
col = ⌊a⌋,      row = ⌊b⌋
fa  = a − col,   fb = b − row
up  = (fa ≥ fb) ? UR : LL
```

### When Recompute Runs
- on `'s'` (set START)
- on `'e'` (set END)
- on `'+/-'` (size change)
- NOT on cursor move alone

### Pool Storage
`PathPool` — flat array of `TPath{col, row, up}`. `path_add` deduplicates via `path_contains`.

### Non-Obvious Decisions
- **Centroid line is shorter** than vertex-to-vertex: lines stay in interior; no edge-tie ambiguity at samples.
- **Sampling step**: `size/4` — fine enough for any angle to never skip a triangle.
- **Markers as separate render pass**: `S` and `E` glyphs draw AFTER the path, then cursor draws on top.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Path pool cap |

### Open Questions
- For an exactly horizontal line of length L pixels, how many triangles does the path contain?
- How does the half-rect path differ from the equivalent equilateral path between the "same" two cells?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ
§4 formula  — pixel_to_tri (axis-aligned), tri_centroid_pixel
§5 pool     — PathPool: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw + S/E markers
§7 path     — path_compute pixel-walk
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
's' → set START → path_compute
'e' → set END   → path_compute
'+/-' → size change → path_compute
frame → grid_draw + path_draw + S/E markers + cursor_draw
```
