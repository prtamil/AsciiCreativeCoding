# Concept: Square Cell Grid

## Pass 1 — Understanding

### Core Idea
A pure cell-space display program: no physics, no pixel coordinates. The entire rendering is a double loop over screen rows and columns, with a Boolean condition deciding whether to draw a character at each position. The grid pattern is a mathematical function of `(sr % ch, sc % cw)`.

### Grid Parameters
SQ_CS=3, cw=SQ_CS*2=6, ch=SQ_CS=3

### Rendering Formula
Same as uniform; 2:1 width:height makes cells visually square

### Data Structures
- No simulation state. Just `GridCtx g` holding geometry constants and a colour theme index.
- All rendering done in `draw_bg()`: two nested loops, `attron/attroff`, `mvaddch`.

### Non-Obvious Decisions
- **`safe_mod(a, b) = ((a%b)+b)%b`** is required for diamond/iso grids where `u = sc - ox` can be negative; C `%` returns negative values for negative inputs.
- **`(chtype)(unsigned char)ch`** double cast on every `mvaddch` prevents sign-extension for char values > 127.
- **Cell-space means one character = one cell**: there is no sub-cell precision and no interpolation.
- **Aspect ratio design**: terminal cells are roughly 2:1 wide:tall. Grid constants are chosen so cells look visually proportional (e.g. cw=6, ch=3 for a square-looking cell).

### Key Constants
| Name | Role |
|------|------|
| `cw` | Cell width in screen columns |
| `ch` | Cell height in screen rows |
| `ox, oy` | Screen origin (centre) — used by diamond/iso/origin grids |

### Open Questions
- How would you add cursor navigation to highlight one cell?
- What changes if you make `ch` odd vs even?
- Can you overlay two grid types (e.g. dot grid on top of brick stagger)?

---

## From the Source

**Algorithm:** Cell-space sweep — iterate every screen position `(sr, sc)`, evaluate the grid condition, draw one character or skip. O(rows × cols) per frame — trivially fast for a terminal screen.

**Math:** The conditions are modular-arithmetic membership tests. For rectangular grids: `sr % ch == 0` tests "is this row a grid line?". For rotated grids (diamond, iso): the affine rotation of screen coordinates into grid coordinates is tested modulo the grid period.

**Rendering:** `erase() → draw_bg() → HUD → wnoutrefresh() → doupdate()`. No physics tick. The `TARGET_FPS=30` limit exists only to prevent a busy loop, not because anything is animating.

---

## Pass 2 — Implementation

### Pseudocode
```
draw_bg(g):
    attron(PAIR_GRID)
    for sr in 0..rows-2:
        for sc in 0..cols-1:
            if grid_condition(sr, sc, g):
                ch = pick_char(sr, sc, g)
                mvaddch(sr, sc, (chtype)(unsigned char)ch)
    attroff(PAIR_GRID)
```

### Module Map
```
§1 config  — cw, ch, ox, oy, colour pair IDs
§3 color   — 5 themes, 256-color with 8-color fallback
§5 draw    — draw_bg() cell-space sweep
§8 app     — main loop: input (q/ESC/+/-) → erase → draw → doupdate → sleep
```

### Data Flow
```
(sr, sc) → grid_condition(sr, sc, cw, ch, ox, oy) → char → mvaddch
```
