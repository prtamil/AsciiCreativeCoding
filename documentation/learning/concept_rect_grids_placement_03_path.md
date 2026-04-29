# Concept: Two-Point Path Drawing (03_path.c)

## Pass 1 — Understanding

### Core Idea
A two-point path editor: set point A, set point B, draw a path between them. The interaction is a **three-state finite automaton** driven by a single key (`p`). Four path algorithms connect the two points: Bresenham straight line, L-shaped rectilinear path, hollow rectangle ring, and diagonal staircase.

### Mental Model

```
SEL_IDLE  →(p)→  SEL_ONE   (A marked on screen)
SEL_ONE   →(p)→  SEL_TWO   (B marked on screen)
SEL_TWO   →(l/j/o/x)→  draw path → SEL_IDLE
SEL_TWO   →(p)→  SEL_IDLE  (cancel, clear A and B)
```

The user sees `'A'` and `'B'` markers at the selected points. Path keys only fire in `SEL_TWO`; pressing `p` in `SEL_TWO` cancels without drawing.

### Key Equations

**Bresenham line** (column-major case, `dc >= dr`):
```
err = 2*dr - dc
for i in 0..dc:
    place(r, c)
    if err > 0: r += sign(r1-r0); err -= 2*dc
    err += 2*dr
    c += sign(c1-c0)
```

**L-path**: horizontal leg `(r0, c0..c1)` then vertical leg `(r0..r1, c1)`.

**Ring**: top row `(r0, cmin..cmax)` + bottom row `(r1, cmin..cmax)` + left/right cols `(rmin+1..rmax-1, c0)` and `(rmin+1..rmax-1, c1)`.

**Diagonal staircase**: `step = (sign(dr), sign(dc))` each iteration until `(r1, c1)` is reached.

### Data Structures

**`Cursor`** extended vs `01_direct`:
```c
typedef struct {
    int r, c;          /* cursor position */
    int ar, ac;        /* point A */
    int br, bc;        /* point B */
    SelState state;    /* SEL_IDLE / SEL_ONE / SEL_TWO */
} Cursor;
```

`cursor_reset` resets state to `SEL_IDLE` as well as position.

### Non-Obvious Decisions

- **Three-state machine, not two**: The original two-state (IDLE/ONE) design had the second keypress cancelling instead of setting B. Adding `SEL_TWO` separated "B is set" from "cancel" — the machine now matches its own documentation.
- **`'p'` for set-point, not ENTER**: ENTER (`'\n'`, `'\r'`, `KEY_ENTER`) behaves inconsistently across terminal emulators (some send `'\n'`, some `'\r'`, some `KEY_ENTER`). A plain lowercase letter is 100% reliable.
- **Path keys `l/j/o/x`** avoid conflict with grid cycle `a`/`e` and do not shadow uppercase variants that have other meanings.
- **State reset on path draw**: After drawing, state returns to `SEL_IDLE` so the user can immediately start a new path without pressing Cancel.
- **`sel_hint()` function**: Returns a context string shown in the top-right HUD — `"p:set-A"`, `"A set — p:set-B"`, `"A+B set — l/j/o/x:draw  p:cancel"`. The HUD teaches the user the current state machine step.

### Key Constants

| Name | Role |
|------|------|
| `SEL_IDLE=0` | No points selected |
| `SEL_ONE=1` | A selected |
| `SEL_TWO=2` | Both A and B selected |

### Open Questions

- Why does Bresenham produce exactly one cell per major-axis step with no gaps or doubles?
- What does an L-path look like on an isometric grid — does the "L" still look like a right angle?
- What is the maximum number of cells a ring can place? What is the minimum?

---

## From the Source

**Algorithm:** Four grid path generators: Bresenham (integer line approximation), L-path (rectilinear routing), ring (border rectangle), diagonal staircase. All use the same `pool_place` interface and the same `ctx_to_screen` coordinate seam as `01_direct`.

**Math:** Bresenham's algorithm (1965, IBM Systems Journal 4(1):25-30) accumulates error in multiples of 2 to avoid floating point: `err` tracks the sub-pixel fractional position of the ideal line. When `err > 0` the minor axis advances.

**Rendering:** Background → objects → A/B markers (`cursor_draw`, via `mark_at`) → cursor → HUD. A and B markers are drawn in `PAIR_ACTIVE|A_BOLD` before the `@` cursor so the cursor overwrites markers if collocated.

---

## Pass 2 — Implementation

### Pseudocode

```
handle_key('p'):
    if state == IDLE: ar=r; ac=c; state=ONE
    elif state == ONE: br=r; bc=c; state=TWO
    else: state=IDLE   // cancel

handle_key('l'):
    if state != TWO: return
    path_line(pool, g, ar, ac, br, bc, '*')
    state = IDLE

path_line(pool, g, r0,c0, r1,c1, glyph):
    dr=|r1-r0|; dc=|c1-c0|
    sr=sign(r1-r0); sc=sign(c1-c0)
    if dc >= dr:          // column-major
        err = 2*dr - dc
        for i=0..dc:
            pool_place(pool, r, c, glyph)
            if err>0: r+=sr; err-=2*dc
            err+=2*dr; c+=sc
    else:                 // row-major (symmetric)
        ...
```

### Module Map

```
§1 config   — grid constants
§4 gridctx  — GridCtx, ctx_to_screen, ctx_draw_bg (same as 01)
§5 pool     — ObjectPool (pool_place, pool_draw, pool_clear)
§6 paths    — path_line, path_lpath, path_ring, path_diagonal
§7 cursor   — Cursor (extended), SelState FSM, cursor_draw, mark_at
§8 scene    — scene_draw + sel_hint()
§9 screen   — screen_init
§10 app     — main loop
```

### Data Flow

```
getch('p') → update SelState, store (ar,ac) or (br,bc)
getch('l/j/o/x') → if SEL_TWO: path_*(pool, g, ar,ac, br,bc) → pool_place per cell
pool_draw → ctx_to_screen → mvaddch
cursor_draw → mark_at('A') / mark_at('B') / mark_at('@')
```
