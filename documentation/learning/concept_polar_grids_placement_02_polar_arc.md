# Concept File: `grids/polar_grids_placement/02_polar_arc.c`

> Two-anchor polar drawing — arc, spoke, ring, and radial; an `AnchorCtx` state machine drives all four shape types.

---

## Pass 1 — First Read

### Core Idea

You place two anchors by pressing `p` twice (IDLE → ONE → TWO), then press a shape key to populate objects along the chosen polar curve. The same anchor mechanism used in `03_path.c` for rectangular grids is adapted here: anchor A defines the starting geometry, anchor B the ending geometry, and the shape function walks the curve between them.

### Mental Model

Think of a compass: you plant the point at anchor A (the pivot) and swing the pencil to anchor B. For an arc, A gives the radius and B gives the angular span. For a spoke, A gives the start angle and B gives the radial endpoint. For a ring, only A matters (full revolution). For a radial, only A's angle matters (full outward ray).

### Key Equations

**Arc** — walk θ from θ_A to θ_B at radius r_A:
```
step = max(CELL_W / (r_A + 1), ARC_STEP_MIN)   /* adaptive: smaller r → coarser step */
for t = theta_A; t <= theta_B; t += step:
    col = ox + round(r_A × cos(t) / CELL_W)
    row = oy + round(r_A × sin(t) / CELL_H)
```

The adaptive step is the key insight: at small radii a fixed angular step would produce overlapping or missing cells. `CELL_W / (r + 1)` ensures roughly one cell per step (arc-length ≈ CELL_W).

**Spoke** — walk r from r_A to r_B at angle θ_A, step = 1 pixel:
```
for r = r_A; r <= r_B; r += SPOKE_PX_STEP:
    col = ox + round(r × cos(theta_A) / CELL_W)
    row = oy + round(r × sin(theta_A) / CELL_H)
```

### Non-Obvious Decisions

- **Pause is capital `P`**: Lowercase `p` advances the anchor state machine (IDLE→ONE→TWO→IDLE). Using lowercase for pause would conflict. Capital `P` is uncommon but unambiguous.
- **4096 object capacity**: Arcs at large radius and rings at medium radius can place hundreds of objects in one call. The rect_grids equivalent uses 256 — polar arcs need more headroom.
- **PAIR_ANCHOR = color 220 (amber)**: Visually distinct from both the grid (cyan/green) and objects (white). Anchors are transient state — they should stand out but not overwhelm.
- **Radial from R_OPS_MIN to half-diagonal**: Avoiding r=0 prevents the cursor getting stuck at origin; half-diagonal reaches the corner regardless of terminal size.

### Open Questions

- What if θ_B < θ_A? Should the arc wrap through 0? (Currently it does not — the arc is drawn only if θ_A ≤ θ_B. Pressing `p` a third time clears anchors, so the user can re-anchor.)
- Could a "sweep" mode draw arcs from r_A to r_B (annular sector) instead of at a fixed radius?

---

## From the Source

```c
typedef enum { IDLE = 0, ONE = 1, TWO = 2 } AnchorState;
typedef struct {
    AnchorState state;
    double r_a, theta_a; int row_a, col_a;   /* anchor A */
    double r_b, theta_b; int row_b, col_b;   /* anchor B */
} AnchorCtx;

static void arc_draw(ObjPool *pool, const AnchorCtx *a, int rows, int cols,
                     int ox, int oy) {
    double step = fmax(CELL_W / (a->r_a + 1.0), ARC_STEP_MIN);
    for (double t = a->theta_a; t <= a->theta_b + step*0.5; t += step) {
        int c, row; polar_to_screen(a->r_a, t, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}
```

---

## Pass 2 — After Running It

### Pseudocode

```
state = IDLE, anchor = {no anchors}
loop:
  key = getch()
  if arrows: move cursor; sync polar
  if 'p':
    IDLE → ONE: set anchor A from cursor
    ONE  → TWO: set anchor B from cursor
    TWO  → IDLE: clear anchors
  if 'l' and state==TWO: arc_draw(pool, anchor)
  if 's' and state==TWO: spoke_draw(pool, anchor)
  if 'r' and state>=ONE: ring_draw(pool, anchor)
  if 'x' and state>=ONE: radial_draw(pool, anchor)
  draw: bg → anchors (@ and #) → objects → cursor → HUD
```

### Module Map

| Function | Shape |
|----------|-------|
| `arc_draw` | Arc at r_A from θ_A to θ_B |
| `spoke_draw` | Radial line from r_A to r_B at θ_A |
| `ring_draw` | Full circle at r_A |
| `radial_draw` | Full ray from R_OPS_MIN outward at θ_A |

### Data Flow

```
'p' key → AnchorCtx advances state, records (r, θ, row, col)
Shape key → shape function walks curve → pool_place each point
cursor_draw renders cursor
anchor display renders '@' at ONE, '#' at TWO
```
