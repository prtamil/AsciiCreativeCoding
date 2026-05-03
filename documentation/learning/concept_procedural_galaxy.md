# Concept — `procedural_galaxy.c`: Logarithmic Spiral Galaxy

(Distinct from the older `concept_galaxy.md`, which covers the orbital-physics spiral-galaxy demo. This file is the procedural / closed-form variant.)

## Core Idea

A galaxy is not stars; a galaxy is a **DENSITY FIELD** over the plane. Where the field is high you sprinkle bright stars; where it's medium you sprinkle dim stars; where it's zero you sprinkle nothing. The shape of the field — bulge plus disk plus arms — is determined by a handful of closed-form Gaussian terms, with the spiral arms coming from one elegant equation: the logarithmic spiral. Add slight noise for organicness, rotate the whole thing slowly for life, and you have a galaxy that fits in 1 KB of code.

---

## The Mental Model

Picture the screen as a piece of black paper. You hold a stencil in front of it — the stencil is opaque except for the shape of the galaxy: a bright disc in the middle, two or four curved slits going outward, and a sparse halo at the edges. You spray-paint white through the stencil. Where the stencil is transparent, lots of paint specks land; where it's nearly opaque, very few do. The resulting picture LOOKS like a galaxy because the stencil shape encodes "a galaxy".

Now replace the stencil with a function. `density(r, θ)` returns a number in `[0, 1]` for any point on the paper — this is the local "transparency". Replace the spray-paint with a hash: for each pixel, roll a fair die and place a star iff the die-roll falls under the density value. The galaxy emerges automatically. Rotate the function (`θ → θ + ωt`) and the galaxy turns.

The **logarithmic spiral** part is the one piece of geometry to internalise. Every point `(r, θ)` on the spiral satisfies:
```
θ = ln(r) / b + 2πk/N
```
for some integer k. The MAGIC of doing math in `(r, θ)` instead of `(x, y)` is that "distance to the nearest arm" becomes a one-line formula: subtract `ln(r)/b` from θ, wrap into one period, multiply by r. That's the arc-length to the arm. Plug into a Gaussian and you have arm density. No iteration, no nearest-point search, no data structure — pure trigonometry.

---

## Algorithm in Steps

1. Each frame, advance the rotation angle: `angle += ω · dt`.

2. For every visible screen cell `(sx, sy)`:
   a. Translate so `(cx, cy)` is the galaxy centre:
      ```
      fx = sx − cx
      fy = (sy − cy) · 2          // aspect: cells are 2× tall
      ```
   b. Rotate INTO the galaxy frame (inverse of screen rotation) by the current angle:
      ```
      gx =  fx·cos(angle) + fy·sin(angle)
      gy = −fx·sin(angle) + fy·cos(angle)
      ```
   c. Normalise to galaxy-radius units:
      ```
      gnx = gx / R0     (R0 = galaxy half-width in cells)
      gny = gy / R0
      ```
   d. Polar:
      ```
      r  = √(gnx² + gny²)
      θ  = atan2(gny, gnx)
      ```
      If `r > DISK_MAX`, this cell is outside the galaxy — skip.
   e. Sample `fBm(gnx, gny, noise_time)` — used to perturb the arm width and (in NEBULA) drive the cloud overlay.
   f. Compute density per the active pattern (see formulas).
   g. Quantise the galaxy-frame coords to integer "world cells", hash, normalise to a uniform fraction `h_unit ∈ [0, 1)`.
   h. If `h_unit < density · STAR_PROB_SCALE` → STAR:
      - pick colour from r-bucket (warm centre → cool edge)
      - pick glyph + brightness from density bucket
      - mvaddch
      Else if pattern == NEBULA and density > 0.05 and fBm > T:
      - pick nebula tint, paint cloud glyph
      Else: leave cell black.

3. Draw HUD, present, sleep.

---

## Key Formulas

**Logarithmic spiral** (one arm, k = 0):
```
r = a · exp(b · θ)            ⇔    θ = ln(r/a) / b
```

**N-arm spiral** — angular distance from `(r, θ)` to the nearest arm:
```
α    = θ − ln(r) / b                                  // raw phase
seg  = 2π / N
α    = α − seg · round(α / seg)        // wrap to [-seg/2, +seg/2]
arc  = α · r                            // arc-length to arm
```

**Arm density** (Gaussian falloff with optional noise modulation):
```
W    = ARM_WIDTH · (1 + 0.2 · (fbm − 0.5) · 2)
arm  = exp(-arc² / W²)
```

**Bulge / disk** (Gaussian envelopes):
```
bulge(r) = exp(-r² / σ_b²)
disk(r)  = exp(-r² / σ_d²)
```

**Bar** (BARRED pattern only) — elliptical core, oriented along x:
```
bar(x, y) = exp(-x² / B_x² − y² / B_y²)
```

**Composite density:**
```
SPIRAL/NEBULA :  bulge + disk · arm
BARRED        :  bulge·0.6 + bar·BAR_AMP + disk·arm·(1−bar)
ELLIPTICAL    :  bulge·0.9 + halo·0.45                  // no θ
```

**Star gate:**
```
h_unit = (hash3(qx, qy, pat) & 0xFFFFFF) / 2²⁴
is_star = h_unit < density · STAR_PROB_SCALE
```

**Rotation** (rigid — every point rotates at the same angular speed):
```
angle' = angle + ω · dt
```

---

## Edge Cases and Pitfalls

- **LOG SINGULARITY.** `ln(r) → −∞` as `r → 0`. Clamp `r` to a small floor (≈ 0.04) before computing the spiral phase, OR let the bulge term dominate at small r so the arm term's behaviour at the singularity doesn't matter. The code does both — clamp AND let bulge dominate.

- **ASPECT RATIO.** Terminal cells are ~2× taller than wide. Without multiplying `(sy − cy)` by 2 in `fy`, the galaxy renders as a flat horizontal ellipse rather than a circle. Always correct in the rendering math. Use `floor(gy/2)` when quantising for the hash so the integer galaxy cells stay roughly square.

- **RIGID VS DIFFERENTIAL ROTATION.** Real galaxy arms are density waves — the stars passing through them rotate at different angular speeds at different radii (Ω(r) ∝ 1/r approximately). If you rotate the density field rigidly (ω constant in r), the visual arms STAY the same shape; if you let ω depend on r, the arms wind up infinitely tight in finite time (the "winding problem" that motivated density-wave theory). For this demo we use rigid rotation — the field IS the arms, so they don't wind.

- **HASH JITTER UNDER ROTATION.** `floor(gx)`, `floor(gy)` jump by 1 as the rotated coord crosses an integer line. So as the galaxy turns, individual stars STEP between adjacent cells rather than sliding smoothly. This is unavoidable in cell-based rendering. The eye reads it as scintillation — actually adds to the "starlight" illusion.

- **PATTERN-DEPENDENT HASH.** We pass the pattern index as the third arg to hash3 so each pattern has its own star arrangement. Otherwise switching from SPIRAL to ELLIPTICAL with the same rotation angle would leave many of the same stars visible — looks like a half-collapsed transition rather than a genuinely different galaxy.

- **OFF-SCREEN R.** After rotation+aspect correction a screen-corner cell can have `r > 1.5`. Skip these immediately (with `r² > some cutoff²`) to save the log/exp work on cells that will never have a star.

- **PROB SCALE.** With density ≈ 1.0 at the bulge centre, the gate `h_unit < density` places stars in EVERY cell at the bulge — there are not enough free cells. Multiply density by a small constant (`STAR_PROB_SCALE ≈ 0.15`) so the *peak* per-cell prob is ~15 %; gives a satisfyingly dense bulge without saturation.

---

## How to Verify

- **Pause** (space): galaxy freezes. Press space again: rotation resumes from the same angle. Verifies fixed-step time.

- Press **`r`**: rotation snaps to angle 0 and the noise re-seeds. The overall galaxy SHAPE is unchanged (same density function), only the specific star positions rearrange. Verifies hash/density separation.

- **SPIRAL**: count arms. Should be 4. The arms should be CURVED, not straight, with a clear sense of "winding" outward. If they look like spokes on a wheel, `SPIRAL_PITCH` is too high — the formula has degenerated into θ = constant lines.

- **BARRED**: a bright bar should run horizontally through the centre (when angle = 0). Two arms should emerge from its END points, not from the centre. If they emerge from the centre, the bar factor isn't suppressing the inner arm region.

- **ELLIPTICAL**: should look like a fuzzy round ball with no visible structure. If you see arms, the pattern dispatch is broken.

- **NEBULA**: in the gaps between visible stars, you should see soft coloured cloud puffs that drift slowly (noise time advancing). The clouds should ONLY appear inside the visible disk, not in the outer halo or background.

- Speed +/-: pressing + should make rotation noticeably faster. Pressing - should slow it down. The user-facing speed value in the HUD updates on each press. Halving the speed should halve the visible angular velocity exactly.

---

## References

- Wikipedia — [Logarithmic spiral](https://en.wikipedia.org/wiki/Logarithmic_spiral).
- Wikipedia — [Spiral galaxy / pitch angle](https://en.wikipedia.org/wiki/Spiral_galaxy).
- Wikipedia — [Density wave theory](https://en.wikipedia.org/wiki/Density_wave_theory) (why arms exist at all).
- Lin & Shu (1964) — "On the spiral structure of disk galaxies", ApJ 140, 646 — the classic density-wave paper.
- Inigo Quilez — "Painting a galaxy" / domain-warping article (https://iquilezles.org/articles/warp/).
- Red Blob Games — Map generation with noise.

---

*Source: `procedural/worldgen/procedural_galaxy.c`*
