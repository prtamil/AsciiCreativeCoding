# Pass 1 — kaboom: Deterministic LCG Explosion

## Core Idea

A single explosion animation plays through 150 frames. The explosion is computed frame-by-frame using a deterministic Linear Congruential Generator (LCG) PRNG — not `rand()`. This means the same seed always produces the same explosion shape. The blast consists of three visual layers built each frame: initial disc, blast wave (shaped by `petal_n` lobes), and 800 3D debris blobs. The animation plays, then restarts with a new theme and shape combination.

---

## The Mental Model

The terminal is treated as a coordinate system centered at (0,0). Each frame, every cell is evaluated by mathematical formulas to decide what character and color to show.

**Three phases:**
1. **Frame 0**: single `*` at center (ignition flash)
2. **Frames 1-7**: expanding disc of `@` — radius = frame × disc_speed
3. **Frames 8-150**: blast wave expands outward; inner fireball characters shrink; 3D blob cloud flies outward

The blast wave shape is controlled by `petal_n` — an angular frequency that creates symmetric lobes (`petal_n=16` = spiky ring, `petal_n=6` = hex star, `petal_n=0` = smooth sphere).

The 800 debris blobs are precomputed at init using the LCG: each blob has a position on a unit sphere surface. Each frame, blobs are scaled by `(frame-6) × blob_speed` and projected with perspective onto the terminal.

---

## Data Structures

### `Blob` struct
| Field | Meaning |
|---|---|
| `x, y, z` | Unit-sphere surface position, scaled by (1.3 + 0.2*prng()) for irregular radii |

### `Cell` struct
| Fields | Meaning |
|---|---|
| `ch` | Character to draw (0 = empty cell) |
| `color` | ColorID (FLASH/INNER/WAVE/BLOB_F/BLOB_M/BLOB_N) |

### `Blast` struct
| Field | Meaning |
|---|---|
| `blobs[800]` | All 3D debris particles — fixed, computed once at init |
| `cells[cols*rows]` | Per-frame output: what to draw where |
| `frame` | Current frame index 0..NUM_FRAMES |
| `theme` | Index into k_themes (color set) |
| `shape` | Index into k_shapes (geometry parameters) |
| `done` | True when frame reaches NUM_FRAMES |

### `BlastShape` struct
Controls the geometry of the explosion:
- `petal_n` — number of lobes in angular modulation (0 = smooth)
- `ripple` — amplitude of lobe wobble
- `disc_speed` — how fast initial disc expands
- `y_squash` — vertical compression of blob cloud
- `persp` — perspective depth
- `blob_speed` — how fast blobs fly out
- `flash_chars` / `wave_chars` — character sequences for each phase

---

## The Main Loop

kaboom plays a pre-scripted animation:
1. `blast_tick()` — render frame N into `cells[]`, then advance `frame++`
2. `blast_draw()` — write non-empty cells to stdscr
3. When `done`, reset with next theme+shape and replay

---

## Non-Obvious Decisions

### LCG PRNG instead of `rand()`
```c
static double prng(void) {
    static long long s = 1;
    s = s * 1488248101LL + 981577151LL;
    return ((s % 65536) - 32768) / 32768.0;
}
```
The LCG state is global and deterministic from `s=1`. Every run produces the same blob positions. This means `'r'` to replay shows the exact same explosion — consistent, reproducible art. No need to store seeds or reset.

### Cell-based rendering (no erase + redraw per cell)
Each frame clears `cells[]` to zero, fills it with character+color for each non-empty cell, then `blast_draw()` writes to terminal. Empty cells (ch==0) are skipped. No `erase()` is called — instead the whole terminal is explicitly overwritten each frame by drawing space characters for cells that had content last frame but are now empty... actually looking at the code, `blast_draw()` skips cells with `ch==0`, but the main loop calls `erase()` before each draw.

### Blob on sphere surface
Each blob is positioned by:
1. Generate random (bx, by, bz) in [-1,1]
2. Normalize to unit sphere surface: `b.x = bx/|b|`
3. Add slight radial jitter: multiply by `1.3 + 0.2*prng()`
4. y_squash applied at render time, not at init

This gives uniformly distributed points on a sphere, not in a ball.

### Blast wave character by `v` index
```
v = frame - (int)r - 7
```
`r` is the distance from center (with perspective correction and lobe modulation). `v` indexes into `wave_chars` string — cells just inside the wave front show earlier chars, cells at the wave front show middle chars, cells outside show nothing. The wave is a moving "curtain" that sweeps outward.

### Color zones by z-depth for blobs
```
bz > persp*0.8  → COL_BLOB_F + '.'   (far blobs, dim)
bz > -persp*0.4 → COL_BLOB_M + 'o'  (mid blobs)
else            → COL_BLOB_N + '@'   (near blobs, bright)
```
Depth-based coloring without a z-buffer — just categorical zones.

---

## State Machine

```
PLAYING (frame 0..149):
  Each tick: blast_render_frame(), frame++
  frame == NUM_FRAMES:
    done = true

DONE:
  Next tick: blast_free(), blast_init() with next theme+shape
  done = false, frame = 0
  → PLAYING
```

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|---|---|---|
| `NUM_FRAMES` | 150 | Total animation length |
| `NUM_BLOBS` | 800 | More = denser debris cloud |
| `PERSPECTIVE` | 50.0 | Default; each shape has its own persp |
| `petal_n` (per shape) | 0–16 | Controls lobe count (0=circle, 16=spiky) |
| `ripple` | 0.0–0.6 | How jagged the lobes are |

---

## Open Questions for Pass 3

1. Try seeding the LCG with `time(NULL)` — how does it change the explosion?
2. What happens with `petal_n=1`? (`petal_n=1` produces an asymmetric teardrop)
3. Can you separate the disc phase from the wave phase visually by changing `disc_speed`?
4. The z-depth color zones use categorical buckets — how would continuous z-based brightness look?
5. `blast_render_frame()` rebuilds `cells[]` from scratch each frame — is there a way to make it incremental?
