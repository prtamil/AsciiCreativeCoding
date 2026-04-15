# Pass 1 — artistic/cymatics.c: Chladni Figures (Cymatics)

## Core Idea

When a metal plate is vibrated at a resonant frequency and covered with sand, the sand collects on the nodal lines — the places where the plate doesn't move. These are Chladni figures. The program computes the standing-wave formula for each mode (m, n), finds the nodal lines analytically, and renders them as bright glowing characters. Between modes, the figure smoothly morphs to the next one.

## The Mental Model

Strike a wine glass and the rim vibrates. The vibrating parts move; certain points stay still — those are the nodes. For a 2-D plate, the nodal lines divide the plate into regions that move up and down alternately (antinodes). Sand slides off the moving regions and accumulates on the still lines. The formula `cos(mπx)cos(nπy) − cos(nπx)cos(mπy)` gives the displacement at every point; where it's zero, the node line is.

## Chladni Formula

For mode `(m, n)` at normalized coordinates `(x, y) ∈ [0, 1]`:

    Z(x,y) = cos(m·π·x)·cos(n·π·y) − cos(n·π·x)·cos(m·π·y)

Properties:
- `Z = 0` at the nodal lines (where sand collects)
- `Z > 0` and `Z < 0` in alternating antinode regions
- Requires `m ≠ n` for a non-trivial pattern; requires `m < n` to avoid duplicates (swapping m,n negates Z — same nodal pattern, opposite sign)

## 20 Mode Pairs

All pairs `(m, n)` with `1 ≤ m < n ≤ 7`:
`(1,2), (1,3), (2,3), (1,4), (2,4), (3,4), (1,5), ...` up to `(6,7)`.

Higher mode numbers produce more complex patterns with more nodal lines. `(1,2)` gives a simple cross-like pattern; `(6,7)` gives a dense lattice.

## Morphing Animation

Instead of hard-cutting between modes, the `Z` value blends linearly between the current and next mode:

```c
float z1 = chladni(x, y, m_cur, n_cur);
float z2 = chladni(x, y, m_next, n_next);
float z  = (1.0f - g_t) * z1 + g_t * z2;   // g_t: 0 → 1
```

`MORPH_SPEED = 0.025f` per tick → `1.0 / 0.025 = 40` ticks → ~1.3 s morph.
`HOLD_TICKS = 120` → ~4 s hold before the next morph begins.

The blended `z` doesn't correspond to a physical resonance — it's a visual interpolation — but the result looks like a smooth transformation between figures.

## Nodal Glow Rendering

Rather than binary nodal/not-nodal rendering, five distance bands around the nodal line `|z| = 0` use progressively fainter characters:

| Band | `|z|` range | Character | Attribute |
|---|---|---|---|
| 0 | < 0.04 | `@` | bold, white CP_NODE |
| 1 | 0.04–0.10 | `#` | bold, CP_POS or CP_NEG |
| 2 | 0.10–0.18 | `*` | normal |
| 3 | 0.18–0.28 | `+` | dim |
| 4 | 0.28–0.40 | `.` | dim |
| — | ≥ 0.40 | (blank) | — |

Cells beyond 0.40 are blank — the antinodes are invisible, making the nodal lines "glow" out of a dark background. The sign of `z` (positive vs negative antinode) determines whether CP_POS or CP_NEG color is used for bands 1–4, giving a visual sense of which regions are "up" and which are "down."

## Four Themes

| Theme | CP_POS | CP_NEG | Character |
|---|---|---|---|
| Classic | cyan 51 | red 196 | Default — high contrast |
| Ocean | blue 45 | teal 30 | Cool |
| Ember | orange 202 | dark-red 160 | Warm |
| Neon | green 82 | magenta 201 | Bright |

All themes use white (xterm 231) for the innermost nodal band.

## Non-Obvious Decisions

### Why `m < n` only?
Swapping `m` and `n` negates Z (`Z(m,n) = −Z(n,m)`). The nodal lines (where Z=0) are identical; only the signs of the antinode regions flip. This is visually equivalent and would be a duplicate. So only `m < n` pairs are included.

### Why morph linearly between Z values rather than between (m,n) parameters?
Interpolating the parameters directly (`m → m+1`) doesn't produce a smooth visual transition because `m` and `n` are discrete integers. Interpolating the `Z` outputs produces a smooth visual blend even though it doesn't correspond to a physical resonance.

### Why hold for 4 seconds before morphing?
Each Chladni figure has a specific structure worth studying. A 4-second pause gives enough time to see the full pattern and count the nodal regions. The morph itself is brief (~1.3 s) so the transition is clear but doesn't dominate the viewing time.

### Why the glow bands instead of just the nodal line?
The exact nodal line `|z| = 0` would be a 1-pixel-thick line, nearly invisible at terminal resolution. The glow bands make the nodal pattern clearly visible and beautiful, while still conveying the mathematical structure: brightest at the node, fading outward.

## Key Constants

| Constant | Effect |
|---|---|
| MORPH_SPEED | Controls morph duration (1/speed = frames to morph) |
| HOLD_TICKS | How long each figure is displayed before morphing |
| NODAL_THRESH | Inner band threshold; smaller → thinner bright core |
| N_MODES | Total number of mode pairs (20 for m<n≤7) |

## From the Source

**Physics/References:** Ernst Chladni, 1787 — the first systematic study of vibration patterns on plates. These are called Chladni figures after him.
**Math:** Resonant frequency scales as `f_mn ∝ √(m² + n²)`. Boundary condition is Neumann (free edge): the normal derivative of displacement is zero at the plate boundary — the plate is free to vibrate, not clamped. The case `m = n` gives `Z = 0` everywhere — a degenerate trivial solution with no pattern.
**Algorithm:** The formula `Z(x,y) = cos(mπx)cos(nπy) − cos(nπx)cos(mπy)` is evaluated per pixel; the nodal glow bands are fixed thresholds on `|Z|`.
