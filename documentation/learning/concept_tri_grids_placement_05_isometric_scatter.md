# Concept: Isometric (Solid-Fill) Distance-Coloured Scatter

## Pass 1 — Understanding

### Core Idea
Two layers: ISO BACKGROUND (every triangle filled by palette_index) and SCATTER OVERLAY (random triangles overlaid with glyphs whose foreground colour comes from a distance-bucket palette). The iso colours decorate the field; the scatter glyphs read the cursor distance.

### Distance Metric (Same As 01)
```
d = |obj.col − cur.col| + |obj.row − cur.row| + (obj.up != cur.up ? 1 : 0)
bucket = min(d, N_BUCKETS − 1)
```

### Palette Conflict Resolution
The bucket palette (warm → cool) must be readable on ANY of the 6 iso fills. The bucket pair uses bright foreground colours chosen for high luminance contrast. Theme cycle re-tunes the bucket palette in lockstep with the iso palette.

### Storage vs Colouring
- **STORAGE**: random scatter once per reseed.
- **COLOURING**: per-frame from cursor distance.

### Non-Obvious Decisions
- **Two palettes coexist**: iso 6-cycle + 6-bucket gradient. Must visually distinguish.
- **Theme cycle preserves scatter addresses**: same dots stay where they are; iso bg changes; bucket overlays remain aligned to cursor distance.
- **No PAIR_CURSOR shortcut**: each bucket has its own pair (PAIR_BUCKET_0..5) so the scatter dots can express the gradient distinctly.

### Key Constants
| Name | Role |
|------|------|
| `SCATTER_RADIUS` | Half-extent of spawn box (cells) |
| `DENSITY_DEFAULT` | Starting count |
| `N_BUCKETS` | 6 — colour gradient stops |
| `N_PALETTE` | 6 — iso fill cycle |

### Open Questions
- How would you blend the iso bg colour and bucket fg colour into a single perceptual gradient?
- Could the scatter SHAPE follow the iso lattice (triangular dots vs single character)?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — SCATTER_RADIUS, DENSITY_DEFAULT, N_BUCKETS, N_PALETTE
§4 formula  — pixel_to_tri + palette_index + centroid
§5 pool     — ScatterPool{col, row, up}: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw
§7 scatter  — random spawn + per-frame bucket
§8 scene    — solid-fill raster + scatter + cursor
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
SPACE / +/- → reseed → fill ScatterPool
frame:
    raster: pixel_to_tri → palette_index → bg fill
    for each scatter entry:
        d      = manhattan + Δup penalty
        bucket = min(d, N_BUCKETS - 1)
        attron(PAIR_BUCKET_0 + bucket)
        mvaddch(centroid_screen, '*')
arrow → cur += TRI_DIR (no reseed)
```
