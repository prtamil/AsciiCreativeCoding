# Concept: Phyllotaxis Sunflower Pattern (05_sunflower.c)

## Pass 1 — Understanding

### Core Idea
Parametric dot placement using Vogel's 1979 model: seed i placed at `(sqrt(i) × SPACING, i × GOLDEN_ANGLE)` in polar coordinates.  The golden angle (≈137.508°) is irrational relative to 2π, ensuring seeds never stack on a spoke.  This is the algorithm behind sunflower heads, pinecones, and daisy centres.

### Mental Model
A lazy Susan (turntable) with a seed dispenser at the rim.  Each tick: rotate by GOLDEN_ANGLE, drop a seed, move the rim outward by a small amount (proportional to √i for uniform density).  Because GOLDEN_ANGLE is irrational relative to 2π, the turntable never returns to the same angle — seeds fill the disc uniformly.

**The 'g' experiment**: Replace GOLDEN_ANGLE with 72° (= 2π/5).  Seeds now form 5 clear spokes with gaps — density is uneven.  This dramatically shows why the golden angle is special.

### Key Equations

**Vogel model (seed i):**
```
r_i = sqrt(i) × SPACING       pixel radius
θ_i = i × GOLDEN_ANGLE        polar angle (radians)
```

**Golden angle:**
```
GOLDEN_ANGLE = 2π / φ²  ≈ 2.3999 rad  ≈ 137.508°
where φ = (1+√5)/2 ≈ 1.618
```
Equivalently: GOLDEN_ANGLE = 2π × (1 − 1/φ) = 2π − 2π/φ.

**Equal-area property of √i spacing:**
```
Area of disc to seed i = π × r_i² = π × i × SPACING²
→ each seed adds area π × SPACING²  (constant, regardless of i)
```
So density is uniform: every unit area contains the same number of seeds.

**Cell conversion (inverse of cell_to_polar):**
```
col = ox + round(r × cos θ / CELL_W)
row = oy + round(r × sin θ / CELL_H)
```

### Non-Obvious Decisions

- **sqrt(i) not i**: Linear spacing would produce dense inner seeds and sparse outer seeds.  The square-root gives uniform area density — each new seed adds the same area.
- **visited[] prevents overwrites**: Multiple seeds can map to the same terminal cell at different parameter values.  Without the boolean visited grid, later seeds would overwrite earlier ones, erasing part of the pattern.
- **Stack-allocated visited[rows][cols]**: At most ~250×80 ≈ 20 000 booleans — safe on the stack.  Heap allocation would require malloc/free per frame.
- **ANGLE_TABLE[3] = 2π(1−1/φ) ≈ 222.5°**: This is the same as GOLDEN_ANGLE modulo 2π (= 360° − 137.508° = 222.492°).  The pattern should look identical to entry [0] — an intentional "same result, different parametrisation" entry for the experiment.

### Open Questions

- Why do the spiral families in a sunflower count 8, 13, 21, 34, 55 … (Fibonacci numbers)?  (Answer: the convergents of the continued fraction for φ give the best rational approximations, which set the spacing of the visible spiral families.)
- What happens if you use π instead of GOLDEN_ANGLE?  (Answer: π is rational ×2 relative to 2π; you get 2 spokes.)
- If SPACING → 0, what does the seed distribution look like?  (Answer: all seeds pile up at the origin — the r_i = √i × SPACING term requires SPACING > 0 for the seeds to spread.)

---

## From the Source

**Algorithm:** Vogel's sunflower model.  Parametric loop over seeds (not screen cells), converting each seed's polar position to a terminal cell.

**Math:** GOLDEN_ANGLE = 2π/φ².  √i spacing for uniform density.  Visited[] prevents double-writes.

**Rendering:** This is the only file in the polar series that uses parametric placement (seed → cell) rather than a screen sweep (cell → test).  The pattern is static per frame — no animation, just recalculate when parameters change.

---

## Pass 2 — Implementation

### Pseudocode

```
grid_draw(rows, cols, ox, oy, spacing, n_seeds, angle):
    visited[rows][cols] = all false
    for i = 0 .. n_seeds:
        r = sqrt(i) × spacing
        θ = i × angle
        col = ox + round(r × cos(θ) / CELL_W)
        row = oy + round(r × sin(θ) / CELL_H)
        if out of bounds: skip
        if visited[row][col]: skip
        visited[row][col] = true
        draw SEED_CHAR at (row, col)
```

### Module Map

```
§1 config   — PHI, GOLDEN_ANGLE, ANGLE_TABLE[5], SPACING_DEFAULT, N_SEEDS
§4 coords   — polar_to_cell (inverse direction — only file to use this)
§5 draw     — grid_draw (parametric seed loop, not screen sweep)
§6 scene    — scene_draw (HUD shows angle name from ANGLE_TABLE)
§8 app      — 'g' cycles angle_idx through ANGLE_TABLE
```

### Data Flow

```
getch('g') → angle_idx = (angle_idx+1) % N_ANGLES
getch('+/-') → adjust spacing
getch('[/]') → adjust n_seeds
scene_draw → grid_draw → for each seed → polar_to_cell → visited check → mvaddch
```
