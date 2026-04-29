# Concept: Logarithmic Spiral Grid (04_log_spiral.c)

## Pass 1 — Understanding

### Core Idea
Spiral arms where the radial gap grows proportionally to the radius — the defining property of logarithmic spirals.  Found in nautilus shells, galaxy arms, and sunflower arrangements.  The golden spiral preset (a ≈ 0.3065) matches Fibonacci phyllotaxis exactly.

### Mental Model
An Archimedean spiral is a spring coiled at constant pitch.  A logarithmic spiral is a spring where each coil is wider than the previous by a fixed factor — like an expanding galaxy arm.  The "equiangular" property: any radial line intersects the spiral at the same angle.

**Key distinction from Archimedean (03):**
- Archimedean: gap is constant in pixels (equal spacing in r)
- Log-spiral: gap grows with radius (equal spacing in ln r)

### Key Equations

**Logarithmic spiral equation:**
```
r = b × e^(a×θ)        →      θ = ln(r/b) / a
```
- `a`: growth rate (radians⁻¹)
- `b = R_MIN`: inner anchor radius (pixels)
- After one turn (Δθ = 2π): r multiplies by `e^(a×2π)`

**N-arm phase test:**
```
θ_norm     = fmod(θ + 2π, 2π)
θ_pred     = ln(r_px / R_MIN) / a           ← angle predicted by spiral equation
raw        = N × (θ_norm − θ_pred)
phase      = fmod(raw + N × 2π, 2π)
on_spiral: phase < SPIRAL_W || phase > 2π − SPIRAL_W
```

**Golden spiral:**
```
a = 2 × ln(φ) / π ≈ 0.3065       φ = (1+√5)/2 ≈ 1.618
```
After one half-turn (π radians): r multiplies by φ² ≈ 2.618.  
After one full turn (2π): r multiplies by φ⁴ ≈ 6.854.

**Equiangular property:**
The angle α between the spiral tangent and the radial direction satisfies `tan α = 1/a`.  For the golden spiral: α ≈ 73°.

### Non-Obvious Decisions

- **θ_pred = ln(r/R_MIN)/a, not ln(r)/a**: The inner anchor b = R_MIN means the phase is zero at r = R_MIN for all θ.  Without the R_MIN normalisation, the inner region produces garbage phase values.
- **Same N×2π trick as 03**: `raw` can be large and negative for small r or large N.  Adding N×2π guarantees positive fmod input.
- **'g' key resets growth to GROWTH_DEFAULT when +/- pressed**: Pressing +/- after enabling golden mode should exit golden mode (the user is now manually adjusting, not in the preset).

### Open Questions

- If a → 0, the log-spiral approaches a circle.  How does that relate to 01_rings_spokes?
- The golden angle (137.5°) from 05_sunflower.c and the golden growth rate here — are they the same φ?  (Answer: yes, both derived from φ, but one is an angle per seed and one is a radial growth rate.)
- What is the self-similar angle α for a = 0.18 (the default)?  (Answer: arctan(1/0.18) ≈ 80°.)

---

## From the Source

**Algorithm:** Same N-arm phase test as 03, with `ln(r/b)/a` replacing `r/a` as the predicted angle.

**Math:** Logarithmic spiral r = b×e^(aθ).  Golden preset: a = 2×ln(φ)/π.  Both Archimedean and log-spiral use the same fmod phase framework — the only difference is the predicted angle formula.

**Rendering:** Same `angle_char(theta)` as 01–03.  The 'g' key demonstrates the golden ratio connection to 05_sunflower.

---

## Pass 2 — Implementation

### Pseudocode

```
grid_draw(rows, cols, ox, oy, growth, n_arms):
    for each cell (row, col):
        (r, θ) = cell_to_polar(col, row, ox, oy)
        if r < R_MIN: skip
        θ_norm  = fmod(θ + 2π, 2π)
        θ_pred  = log(r / R_MIN) / growth
        phase   = fmod(n_arms × (θ_norm − θ_pred) + n_arms × 2π, 2π)
        if on_spiral: draw angle_char(θ)
```

### Module Map

```
§1 config   — GROWTH_DEFAULT/MIN/MAX/STEP, PHI, GROWTH_GOLDEN, R_MIN
§5 draw     — grid_draw (log() replaces linear r/a from 03)
§6 scene    — scene_draw (HUD shows a value + "(golden)" label)
§8 app      — 'g' toggles golden preset; +/- exits golden mode
```

### Data Flow

```
getch('g')   → golden = !golden; growth = GROWTH_GOLDEN or GROWTH_DEFAULT
getch('+/-') → adjust growth; golden = false
scene_draw → grid_draw → log-spiral phase test → mvaddch
```
