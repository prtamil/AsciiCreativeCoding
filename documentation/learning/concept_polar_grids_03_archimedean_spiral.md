# Concept: Archimedean Spiral Grid (03_archimedean_spiral.c)

## Pass 1 — Understanding

### Core Idea
Spiral arms where the radial gap between successive passes is constant in pixels.  The N-arm phase trick collapses all N arms into a single modular test — no loop over arm indices.

### Mental Model
A clock hand rotating at constant angular speed while its tip moves outward at constant speed.  The tip traces an Archimedean spiral.  The pitch (radial advance per full turn) is always the same — this is what distinguishes it from the log-spiral where the gap grows with radius.

**Key distinction:**
- Archimedean: equal gap in pixels at every radius
- Log-spiral (04): equal gap in log(r) — i.e., equal multiplicative factor

### Key Equations

**Archimedean spiral equation:**
```
r = a × θ        one arm
a = PITCH / (2π)  so one full turn (Δθ=2π) advances by PITCH pixels
```

**N-arm phase test:**
```
θ_norm = fmod(θ + 2π, 2π)
raw   = N × (θ_norm − r_px / a)
phase = fmod(raw + N × 2π, 2π)
on_spiral: phase < SPIRAL_W || phase > 2π − SPIRAL_W
```

**Why it detects all N arms**: For point on arm k, θ = r/a + k×2π/N, so  
`N × (θ − r/a) = N × k × 2π/N = k × 2π`.  
`fmod(k×2π, 2π) = 0` for any integer k.  All N arms map to phase ≈ 0.  ✓

**Why add N×2π before fmod?**  
`raw` is unbounded and can be very negative.  Adding N×2π guarantees the argument to fmod is positive, so fmod returns the correct mathematical remainder.

### Non-Obvious Decisions

- **N×2π addition before fmod**: Without it, fmod of a large negative number gives a negative result.  C's fmod is not "mathematical mod" for negatives.
- **SPIRAL_W in N×phase space, not pixel space**: The half-width is measured in the transformed phase space (0 to 2π).  A larger N stretches the phase space, so a given SPIRAL_W corresponds to finer angular discrimination per arm.
- **MIN_R centre suppression**: Near the origin, all N arms converge.  The dense overlapping lines produce a smear; MIN_R removes it.

### Open Questions

- For N=1 and PITCH=32, what is the radial gap between successive visible lines?  (Answer: exactly 32 pixels.)
- What happens visually when N = 6 and PITCH is small?  (Answer: closely packed pinwheel of 6 arms.)
- If SPIRAL_W → π (half of 2π range), what fraction of the screen is covered?  (Answer: 100% — every cell is "on" a spiral.)

---

## From the Source

**Algorithm:** Screen-sweep with N-arm phase test.  O(rows × cols) per frame.

**Math:** Archimedean spiral r = aθ rearranged to test how close a cell is to any arm.  Multiplying by N folds all N arms into a single test near phase = 0.

**Rendering:** `angle_char(theta)` gives the tangent character at each spiral point, producing curved-line appearance rather than dots.

---

## Pass 2 — Implementation

### Pseudocode

```
grid_draw(rows, cols, ox, oy, pitch, n_arms):
    a = pitch / (2π)
    for each cell (row, col):
        (r, θ) = cell_to_polar(col, row, ox, oy)
        if r < MIN_R: skip
        θ_norm = fmod(θ + 2π, 2π)
        phase = fmod(n_arms × (θ_norm − r/a) + n_arms × 2π, 2π)
        if phase < SPIRAL_W || phase > 2π − SPIRAL_W:
            draw angle_char(θ)
```

### Module Map

```
§1 config   — PITCH_DEFAULT/MIN/MAX/STEP, SPIRAL_W, N_ARMS_DEFAULT/MIN/MAX
§4 coords   — cell_to_polar, angle_char (identical to 01/02)
§5 draw     — grid_draw with N-arm phase test
§6 scene    — scene_draw (HUD shows pitch + arm count)
§8 app      — keys: +/- pitch, [/] arm count
```

### Data Flow

```
getch('+/-/[/]') → update pitch / n_arms
scene_draw → grid_draw → phase test per cell → mvaddch(angle_char)
```
