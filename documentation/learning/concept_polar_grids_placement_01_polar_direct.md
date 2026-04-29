# Concept File: `grids/polar_grids_placement/01_polar_direct.c`

> Polar cursor placement editor — arrow-key navigation in screen or polar mode; space places/removes objects on any of 7 polar backgrounds.

---

## Pass 1 — First Read

### Core Idea

A cursor holds **two representations simultaneously**: `(r, θ)` in polar space and `(row, col)` in terminal cell space. The user toggles between **screen mode** (Δrow/Δcol arrows) and **polar mode** (Δr/Δθ arrows). After each move, one representation is recomputed from the other to keep them in sync.

This dual-representation pattern is the foundation of every polar placement editor: objects are stored in cell space (what ncurses can draw), but the user thinks and navigates in polar space.

### Mental Model

Think of a GPS with two coordinate displays: latitude/longitude and grid-reference. Moving by street (screen mode) updates both; moving by bearing (polar mode) also updates both. The conversion is the seam — get it wrong and the cursor "jumps".

### Key Equations

**Screen → polar** (after Δrow/Δcol move):
```
dx = (col - ox) × CELL_W
dy = (row - oy) × CELL_H
r  = sqrt(dx² + dy²)
θ  = atan2(dy, dx)
```

**Polar → screen** (after Δr/Δθ move):
```
col = ox + round(r × cos(θ) / CELL_W)
row = oy + round(r × sin(θ) / CELL_H)
```

The `CELL_W / CELL_H` divisors correct for the non-square character cell aspect ratio. Without them, "circles" appear as ellipses.

### Non-Obvious Decisions

- **R_POLAR_MIN = 2.0**: After a polar-mode move, if `r < R_POLAR_MIN` the cursor would converge on the origin and get stuck (θ becomes undefined near r=0). The clamp prevents this.
- **pool_toggle swap-last**: Removal swaps the target object with the last slot and decrements count — O(1) delete without shifting the array. Order is not preserved, which is fine for unordered point sets.
- **Sunflower background uses `calloc/free` per frame**: The visited-cell array `bool visited[rows×cols]` cannot be a VLA inside `draw_polar_bg` (stack overflow on large terminals). Heap allocation is safe and at 30fps costs ~1ms.

### Open Questions

- What happens if you move in polar mode while near θ = ±π boundary? Does `atan2` discontinuity cause a jump? (Answer: no — the cursor moves in (r, θ) space continuously; `atan2` is only called after screen-mode moves.)
- Could you add a "radial snap" mode where r is quantised to the nearest ring of the active background?

---

## From the Source

Key structs and functions:

```c
typedef struct {
    double r, theta;   /* polar position: pixels and radians */
    int    row, col;   /* terminal cell derived from (r, theta) */
    bool   polar_mode; /* true = Δr/Δθ arrows; false = Δrow/Δcol */
} Cursor;

/* After screen-mode move: recompute polar from cell */
static void cur_sync_screen(Cursor *c, int ox, int oy) {
    cell_to_polar(c->col, c->row, ox, oy, &c->r, &c->theta);
    if (c->r < R_POLAR_MIN) c->r = R_POLAR_MIN;
}

/* After polar-mode move: recompute cell from polar */
static void cur_sync_polar(Cursor *c, int ox, int oy, int rows, int cols) {
    polar_to_screen(c->r, c->theta, ox, oy, &c->col, &c->row);
    /* clamp to screen bounds */
}
```

The `draw_polar_bg` switch inlines all 7 background types; type 4 (sunflower) calls `calloc(rows*cols, 1)` and `free` each frame.

---

## Pass 2 — After Running It

### Pseudocode

```
init cursor at (ox, oy), r=0, theta=0, polar_mode=false
loop:
  key = getch()
  if arrows:
    if polar_mode: adjust r or theta; cur_sync_polar()
    else:          adjust row or col;  cur_sync_screen()
  if 'm': toggle polar_mode
  if space: pool_toggle(row, col)
  if 'a'/'e': cycle bg_type
  draw: erase → draw_polar_bg → pool_draw → cursor_draw('@') → HUD
```

### Module Map

| Function | Role |
|----------|------|
| `cell_to_polar` | Screen → polar conversion (§4) |
| `polar_to_screen` | Polar → screen conversion (§4) |
| `cur_sync_screen` | Sync after screen-mode move |
| `cur_sync_polar` | Sync after polar-mode move + clamp |
| `pool_toggle` | O(1) place/remove at (row, col) |
| `draw_polar_bg` | 7-type background switch |
| `cursor_draw` | Render `@` at cursor position |

### Data Flow

```
Key input
  → if arrows: update Cursor.(r,θ) or Cursor.(row,col)
  → sync function updates the other representation
  → if space: pool_toggle(Cursor.row, Cursor.col)
  → draw_polar_bg(bg_type) renders grid
  → pool_draw renders all placed objects
  → cursor_draw renders '@'
```
