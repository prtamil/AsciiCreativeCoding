# Concept: Logarithmic Polar Grid (02_log_polar.c)

## Pass 1 — Understanding

### Core Idea
Rings placed at exponentially growing radii: `r_k = R_MIN × RATIO^k`.  Each ring is a fixed multiplicative factor further from the centre than the previous.  This log-polar spacing is the coordinate system of the human retina, SIFT feature descriptors, and conformal optics.

### Mental Model
Imagine zooming into a photo: each zoom level doubles the distance.  The log-polar grid formalises this — equal spacing in log space means each ring represents the same multiplicative factor in distance.  Inner rings are dense (fine resolution near origin); outer rings are wide apart (coarse resolution far out).

**KEY DIFFERENCE FROM 01_rings_spokes:**
- 01: `r_k = k × 20 px`  — ring spacing CONSTANT in pixels
- 02: `r_k = R_MIN × RATIO^k`  — ring spacing CONSTANT in log space

### Key Equations

**Log-ring index** (continuous):
```
u = ln(r_px / R_MIN) / LOG_STEP        LOG_STEP = ln(RATIO)
```
u = 0 at r = R_MIN, u = 1 at r = R_MIN × RATIO, u = 2 at next ring, etc.

**Ring detection** (all rings at once):
```
frac = u − floor(u)                     fractional part ∈ [0, 1)
on_ring: frac < RING_W_U || frac > 1 − RING_W_U
```
RING_W_U = 0.08 is a fractional threshold in log-ring-index space.

**Why fractional width, not pixel width?**
At small r, rings are very close together.  A fixed pixel width RING_W would make inner rings too thin to see (or merge entirely).  A fractional width means every ring occupies 8% of its spacing — visually equal in log space.

**Spoke detection**: identical to 01_rings_spokes.

### Non-Obvious Decisions

- **RING_W_U not RING_W**: A fixed pixel width for rings works for linear spacing (01) but fails for log spacing — inner rings would be imperceptibly thin.  A fractional width in log-ring-index space keeps all rings the same visual thickness relative to their spacing.
- **LOG_STEP vs RATIO**: The file stores LOG_STEP (= ln RATIO) internally and derives RATIO = exp(LOG_STEP) for display.  This is because the ring test uses ln() directly — storing LOG_STEP avoids computing log(ratio) inside the inner loop.
- **R_MIN clamp**: Rings below R_MIN are suppressed because `ln(r/R_MIN)` is negative for r < R_MIN, producing an invalid ring index.

### Open Questions

- How does the log-polar transform relate to SIFT feature descriptors?  (Answer: scale changes in Cartesian space become translations in log-polar space — scale invariance becomes shift invariance.)
- Why does `LOG_STEP=0.25` give e^0.25 ≈ 1.28× per ring?  (Just the definition: RATIO = e^LOG_STEP.)
- What is the visual effect of LOG_STEP → 0?  (Rings converge to linear spacing; the outer rings would all collapse together at the pixel scale.)

---

## From the Source

**Algorithm:** Same screen-sweep as 01 with `log()` replacing the linear ring test.  O(rows × cols) per frame, adds one log() per cell.

**Math:** Log-polar transform maps (r, θ) → (ln r, θ), turning circles into horizontal lines.  Scale changes in Cartesian space become translations in log space — the basis of scale-invariant feature descriptors.

**Biological motivation:** Human fovea has approximately log-polar sampling density — the reason peripheral vision is coarser than central vision.

---

## Pass 2 — Implementation

### Pseudocode

```
grid_draw(rows, cols, ox, oy, log_step, n_spokes):
    spoke_angle = 2π / n_spokes
    for each cell (row, col):
        (r, θ) = cell_to_polar(col, row, ox, oy)
        on_ring = false
        if r > R_MIN:
            u = log(r / R_MIN) / log_step
            frac = u − floor(u)
            on_ring = frac < RING_W_U || frac > 1 − RING_W_U
        θ_norm = fmod(θ + 2π, 2π)
        spoke_phase = fmod(θ_norm, spoke_angle)
        on_spoke = r > SPOKE_MIN_R && (spoke_phase < SPOKE_W || …)
        if on_ring && on_spoke: draw '+'
        elif on_ring or on_spoke: draw angle_char(θ)
```

### Module Map

```
§1 config   — R_MIN, LOG_STEP_DEFAULT/MIN/MAX/DELTA, RING_W_U, N_SPOKES
§4 coords   — cell_to_polar (identical to 01)
§5 draw     — grid_draw (log ring test replaces linear test)
§6 scene    — scene_draw (HUD shows ratio = exp(log_step))
```

### Data Flow

```
getch('+/-') → update log_step
scene_draw → grid_draw → log ring test per cell → mvaddch
HUD: ratio = exp(log_step) displayed
```
