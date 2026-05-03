# Concept — `quasicrystal.c`: Plane-Wave Interference Quasicrystals

## Core Idea

To make a pattern with rotational symmetry but no translational periodicity, sum cosine waves in N evenly-spaced directions around the circle. If N is coprime with the lattice symmetries (1, 2, 3, 4, 6), the resulting interference cannot tile periodically — yet it still has perfect rotational symmetry. **That is a QUASICRYSTAL: long-range orientational order without a unit cell.**

---

## The Mental Model

Drop a stone in a still pond and you get circular ripples. Drop two stones at opposite ends and where the ripple-fronts meet, you get an interference pattern — bright bands where the waves add, dark bands where they cancel. Now imagine N stones dropped in a perfect circle around the pond, all at the same moment. Each stone emits a wave; the waves all reach the pond centre simultaneously and cross at angles `2π/N` apart.

Look down at the surface from above. The interference pattern has N-fold (or 2N-fold) rotational symmetry — rotate it by `2π/N` and it looks the same. But for "weird" N like 5, 7, 11, it CANNOT be the result of stamping a single tile periodically; the symmetry is incompatible with any wallpaper group. The pattern is LITERALLY EVERYWHERE, infinite, intricate, ordered, yet has no repeating unit. That is a quasicrystal.

Now let each stone's ripple drift at its own slow rate. The pattern is still N-fold symmetric at every moment, but it MORPHS — stars shift, troughs deepen, new constellations of peaks form and dissolve. That is the animation.

---

## Algorithm in Steps

1. **CHOOSE N** (the wave count). Precompute `wave_cos[m]`, `wave_sin[m]` for `m = 0..N-1` with angle `m·π/N`.

2. **EACH FRAME:**
   a. Advance global time `t`.
   b. For every screen cell `(sx, sy)`:
      - `x = sx;  y = sy · ASPECT_Y` (aspect correction)
      - `intensity = 0`
      - For each wave `m`:
        ```
        wx     = x · wave_cos[m] + y · wave_sin[m]
        φ_m    = (t + offset) · (r_base + m · r_delta)
        intensity += cos( ω·wx + φ_m )
        ```
      - `intensity /= N`  (→ ≈ `[-1, +1]`)
      - Map intensity → glyph + colour via active `GlyphSet`.
      - `mvaddch(sy, sx, glyph)` in selected pair + attr.

3. HUD on top.

---

## Key Formulas

**Wave vector for wave m:**
```
θ_m   = m · π / N                  ω = 2π / λ
k̂_m  = (cos θ_m, sin θ_m)
```

**Intensity at `(x, y, t)`:**
```
I = (1/N) · Σ_{m=0..N-1}  cos( ω · k̂_m · (x, y) + φ_m(t) )
```

**Per-wave phase rate** (gives morphing animation):
```
φ_m(t) = t · (r_base + m · r_delta)
```

**Intensity → ramp level (RAMP glyph):**
```
level = clamp(⌊(I + 1)·4⌋, 0, 7)
```

**Zero-crossing (CONTOUR glyph):**
```
drawn when |I| < ε ; brighter when closer to zero.
```

**Bipolar (WAVES glyph):**
```
I > +T_high → core peak '#'
I > 0       → mid peak  '*'
I > -T_high → mid trough '.'
else         → core trough ','
```

---

## Why N=5, 7, 11 Give Aperiodic Patterns

The **Crystallographic Restriction Theorem** says only rotational symmetries of order 1, 2, 3, 4, 6 can arise in 2-D periodic lattices. A cosine-sum of N waves at `θ_m = m·π/N` has 2N-fold rotational symmetry; for N coprime with `{1, 2, 3, 4, 6}` the resulting symmetry order (10, 14, 22 etc.) cannot be a Bravais lattice — the pattern necessarily has long-range order without translational periodicity, the defining property of a quasicrystal.

- **N=3**: 6-fold → periodic (hexagonal). Useful as the contrast pattern in this demo.
- **N=5**: 10-fold → quasicrystal (Penrose-flavour stars).
- **N=7**: 14-fold → quasicrystal, denser fine structure.
- **N=11**: 22-fold → very intricate quasicrystal.

---

## Edge Cases and Pitfalls

- **N COPRIME RULE.** Periodic vs aperiodic depends on N. N = 3 and N = 6 give visually-perfect HEXAGONAL periodic patterns — useful for contrast in the demo, but they are NOT quasicrystals. The rule: N must be coprime with the divisors of 12 (for 2-D Bravais). 5, 7, 11 work; 3 doesn't.

- **COSINE COUNT AT N=11.** 11 cosines per cell × 19 200 cells × 60 fps ≈ 12.7 M cos calls/s. Modern CPUs handle it (~6 % of one core). If you push N to 31 just to see what happens, expect framerate to drop noticeably.

- **ASPECT CORRECTION.** Multiply screen y by `ASPECT_Y_F` (=2) when sampling so the pattern's circles look round on terminals where cells are 2× taller than wide. Without it, the quasicrystal stars become horizontal ovals.

- **PHASE-RATE SPREAD.** If all waves drift at the SAME rate, the pattern just translates rigidly across the screen — no morphing. A small per-wave delta (`k · 0.07`) makes the relative phases shift, so the pattern visibly REORGANISES, which is much more interesting.

- **CONTOUR THRESHOLD AT N=11.** With 11 waves the intensity field has very fine-scale structure; `|I|<0.05` leaves visible gaps in the contour. For higher N, widen the threshold (or use a derivative-based contour) so the lines stay continuous.

- **RANDOMISE PHASE.** `r` adds a random offset to t. The drift continues from there, so the offset persists across frames. Don't subtract the offset on the next frame — that would snap it back.

---

## How to Verify

- **PAUSE** (space). Pattern freezes. Resume: drift continues from exactly where it stopped.

- **PENTA** pattern. Look for 10-pointed stars (5-fold symmetry, but cosine doubles it). Counting points around the brightest peak near the centre should give 10. The Penrose-tiling kinship is visible.

- **HEPTA** pattern. Stars now have 14 points. The pattern is visibly denser and finer than PENTA at the same wavelength.

- **UNDECA** pattern. Visual density makes individual stars hard to count, but rotational symmetry is still perfect about any centre. Pause and rotate your head 360°/22 ≈ 16.4° — the pattern looks the same.

- **TRI** (N=3) pattern. You'll see HEXAGONAL periodic structure — cells repeat in a tilable lattice. NOT a quasicrystal; included for comparison so the difference is visible when you switch to PENTA.

- **CONTOUR** glyph. The drawn lines should form a network of closed curves — these are the level sets `I = 0` of the cosine sum.

- **PEAKS** glyph. Should show only the bright cells (positive intensity). Half the screen (where `I < 0`) is blank. The visible "stars" are the wave crests.

---

## References

- Shechtman, D. et al. (1984) — "Metallic Phase with Long-Range Orientational Order and No Translational Symmetry", Phys. Rev. Lett. 53(20):1951. Original quasicrystal observation; Shechtman's 2011 Nobel Prize.
- Wikipedia — [Quasicrystal](https://en.wikipedia.org/wiki/Quasicrystal).
- Wikipedia — [Crystallographic Restriction Theorem](https://en.wikipedia.org/wiki/Crystallographic_restriction_theorem).
- Penrose, R. (1974) — "The Role of Aesthetics in Pure and Applied Mathematical Research", Bulletin of the IMA 10:266. The Penrose tiling.
- Mike Bostock — Quasicrystals interactive (https://bl.ocks.org/mbostock/3019563).

---

*Source: `procedural/patterns/quasicrystal.c`*
