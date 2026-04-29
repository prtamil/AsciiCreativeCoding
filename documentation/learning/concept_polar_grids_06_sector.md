# Concept: Equal-Area Sector Grid (06_sector.c)

## Pass 1 — Understanding

### Core Idea
Rings placed at `r_k = sqrt(k) × R_UNIT` so that every annular band (between consecutive rings) has the same area.  Combined with uniform angular sectors, every cell covers the same area — an equal-area polar grid.  This is the same principle used in HEALPix astronomy maps and dartboard probability tables.

### Mental Model
A dartboard where you want each ring to be equally likely to be hit by a dart thrown at random.  Linear rings (01) make outer rings larger, so they're hit more often.  To equalise hit probability, each ring must enclose the same area — so `r_k ∝ sqrt(k)`.

**Comparison:**
| Grid | Spacing law | What's equal |
|------|------------|--------------|
| 01 rings_spokes | `r_k = k × SPACING` | gap in pixels |
| 02 log_polar | `r_k = R_MIN × RATIO^k` | gap in log(r) |
| 06 sector | `r_k = sqrt(k) × R_UNIT` | annular area |

### Key Equations

**Equal-area ring positions:**
```
r_k = sqrt(k) × R_UNIT        k = 1, 2, 3, …
```

**Annular area between ring k and ring k+1:**
```
π × r_{k+1}² − π × r_k² = π × R_UNIT² × ((k+1) − k) = π × R_UNIT²   (constant)
```

**Ring detection** (continuous ring index `k_float = (r/R_UNIT)²`):
```
k_float = (r_px / R_UNIT)²
frac    = k_float − floor(k_float)
on_ring: frac < RING_W_F || frac > 1 − RING_W_F
```
RING_W_F is a fractional threshold in `r²/R_UNIT²` space.

**Visual ring width grows with radius**: A fixed Δ(k_float) = RING_W_F corresponds to:
```
Δr ≈ Δ(r²) / (2r) = (RING_W_F × R_UNIT²) / (2r)
```
Wait — this decreases with r, meaning inner rings would look thicker!  
Actually: `k_float = r²/R_UNIT²`, so `d(k_float)/dr = 2r/R_UNIT²`.  A fixed Δ(k_float) = RING_W_F maps to:
```
Δr = Δ(k_float) / (2r/R_UNIT²) = RING_W_F × R_UNIT² / (2r)
```
This shrinks as r grows — inner rings appear thicker in pixels.  RING_W_F is chosen small enough (0.06) that all rings remain visible.

### Non-Obvious Decisions

- **`k_float = (r/R_UNIT)²` not `r/R_UNIT`**: The square-root spacing means rings are at equal intervals of k_float = (r/R_UNIT)², so the fractional part of k_float detects all rings simultaneously.  If you used `r/R_UNIT`, fmod would detect equally-spaced linear rings (like 01), not equal-area rings.
- **`r_unit_sq` precomputed outside the inner loop**: The product R_UNIT² is constant for the frame.  Precomputing it avoids one multiplication per cell.
- **Sector test identical to 01**: Equal-area sectors (same angular width) are already the default for uniform angular division.  The sector width in radians is constant, so the fmod test from 01 applies unchanged.

### Open Questions

- HEALPix uses equal-area pixels on a sphere.  How does its ring spacing compare to `sqrt(k)`?  (Answer: HEALPix rings also follow a sqrt-like spacing but adapted to the sphere's sin(latitude) metric.)
- If you combine equal-area rings AND equal-area sectors, is each cell truly equal area?  (Answer: yes — annular area is π×R_UNIT² per ring, divided equally among N_SECTORS sectors = π×R_UNIT²/N_SECTORS per cell.)
- What happens to RING_W_F at very large r?  (Answer: Δr → 0 so rings become sub-pixel and invisible — the outer region gradually becomes empty.)

---

## From the Source

**Algorithm:** Screen-sweep with `k_float = (r/R_UNIT)²` ring test.  Identical structure to 01 except the ring detection formula.

**Math:** Equal annular areas follow from r_k = sqrt(k) × R_UNIT.  Continuous ring index is k_float = (r/R_UNIT)² so standard fmod fractional test works.

**References:** HEALPix astronomy tessellation, dartboard probability, equal-area map projections.

---

## Pass 2 — Implementation

### Pseudocode

```
grid_draw(rows, cols, ox, oy, r_unit, n_sectors):
    r_unit_sq = r_unit × r_unit
    sector_angle = 2π / n_sectors
    for each cell (row, col):
        (r, θ) = cell_to_polar(col, row, ox, oy)
        if r < R_MIN: skip
        k_float = (r × r) / r_unit_sq
        frac = k_float − floor(k_float)
        on_ring = frac < RING_W_F || frac > 1 − RING_W_F
        θ_norm = fmod(θ + 2π, 2π)
        sector_phase = fmod(θ_norm, sector_angle)
        on_sector = r > SECTOR_MIN_R && (sector_phase < SECTOR_W || …)
        if on_ring && on_sector: draw '+'
        elif on_ring or on_sector: draw angle_char(θ)
```

### Module Map

```
§1 config   — R_UNIT_DEFAULT/MIN/MAX/STEP, RING_W_F, N_SECTORS, SECTOR_W
§5 draw     — grid_draw (k_float = r²/R_UNIT² ring test, sector test from 01)
§6 scene    — scene_draw (HUD shows R_unit and sector count)
§8 app      — keys: +/- R_unit, [/] sector count
```

### Data Flow

```
getch('+/-') → adjust r_unit
getch('[/]') → adjust n_sectors
scene_draw → grid_draw → equal-area ring test + sector test → mvaddch
```
