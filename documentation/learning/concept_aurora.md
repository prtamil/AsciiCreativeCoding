# Concept: Aurora Borealis (Sinusoidal Curtains)

## Pass 1 — Understanding

### Core Idea
Simulate the visual appearance of the aurora borealis using layered sinusoidal waves. Each "curtain" of light is a band of sine waves with different amplitudes, frequencies, and phases that drift over time. Multiple octaves of sine waves create the rippling, shimmering quality.

### Mental Model
The aurora looks like glowing curtains of light rippling in the night sky. Each curtain is a tall column whose horizontal position oscillates like a wave. Multiple overlapping curtains with slightly different speeds create the shimmering, folding effect.

### Key Equations
```
x(row, t) = Σ_k A_k · sin(ω_k · row + φ_k · t + ψ_k)
```
Where:
- A_k: amplitude of each octave (decreasing)
- ω_k: spatial frequency (increasing with k)
- φ_k: time evolution rate
- ψ_k: initial phase offset

### Data Structures
- N_BANDS aurora bands, each with color (green, cyan, magenta gradient)
- N_OCTAVES sine components per band
- Per-band: `{amplitude[], omega[], phi[], psi[], color_pair}`
- `g_time`: accumulated float

### Non-Obvious Decisions
- **Multi-octave overlay**: One sine wave is too regular. Sum 3–5 octaves with halving amplitude and increasing frequency to get natural waviness.
- **Vertical extent per band**: Each band occupies a vertical strip. The column (x-position) oscillates; all rows of the band draw at that x.
- **Color gradient**: Real aurora fades at top and bottom. Apply brightness falloff based on distance from band center row.
- **Phase drift**: Each octave has a different drift rate `φ_k`. This makes the curtain appear to breathe and shift rather than oscillate mechanically.
- **Transparency effect**: Use dim/normal/bold attributes for inner, mid, and bright regions of the curtain.

## From the Source

**Algorithm:** Analytic wave superposition — stateless. Each frame evaluates brightness and color analytically at every cell. No time-stepping accumulation; previous frame state is not used. This contrasts with grid-based simulation approaches.

**Math:** Aurora formula per cell (col, row): `x = col / cols × 2π`; primary wave `amp1 = sin(x + t·s1)`; harmonic `amp2 = sin(2x + t·s2)`; height envelope `H = sin(row/rows × π)` (curtain boundary). The two sine components run at different time speeds `s1`, `s2`, creating the non-repeating shimmering appearance.

**Data-structure:** Star background uses a deterministic hash instead of a stored array: `star = (col×1003 + row×997 + col×row) % STAR_DENSITY == 0`. Threshold is `STAR_THRESH = 5` out of 256 (~2% of background cells). Zero memory overhead for the star field.

---

### Key Constants
| Name | Role |
|------|------|
| N_BANDS | number of aurora curtain strips |
| N_OCT | number of sine octaves per band |
| AMP_BASE | base amplitude in columns |
| DRIFT_RATE | time scale for animation |

### Open Questions
- What ratio of frequencies creates the most natural-looking curtain?
- Can you add a star field behind the aurora using random static dots?
- The real aurora has magnetic field line structure. Can you visualize that?

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `a.time` | `float` | 4 B | monotonically advancing animation time parameter |
| `a.paused` | `bool` | 1 B | pauses time advancement when true |

---

## Pass 2 — Implementation

### Pseudocode
```
init_band(b):
    b.center_col = cols * (b.index + 0.5) / N_BANDS
    b.center_row = rows / 2
    for k in 0..N_OCT:
        b.amp[k]   = AMP_BASE / (k+1)²    # decreasing
        b.omega[k] = BASE_FREQ * (k+1)    # increasing
        b.phi[k]   = random_rate()
        b.psi[k]   = random_phase()

draw_band(b, t):
    for row in 0..rows:
        # compute curtain x-offset via summed sines
        dx = 0
        for k in 0..N_OCT:
            dx += b.amp[k] * sin(b.omega[k]*row + b.phi[k]*t + b.psi[k])
        col = (int)(b.center_col + dx)
        if col out of bounds: continue
        # brightness falloff from band center
        dist = abs(row - b.center_row)
        bright = 1 - (dist / (rows/2))²
        if bright < 0: continue
        attr = bright > 0.7 ? A_BOLD : (bright > 0.3 ? A_NORMAL : A_DIM)
        attron(COLOR_PAIR(b.color) | attr)
        mvaddch(row, col, '|')

main_loop:
    for each frame:
        t += dt
        erase()
        for each band: draw_band(band, t)
        draw_HUD()
        refresh
```

### Module Map
```
§1 config    — N_BANDS, N_OCT, AMP_BASE, DRIFT_RATE
§2 init      — init_band() for each band
§3 draw      — draw_band() with multi-octave sine + brightness
§4 app       — main loop, keys (speed, band count, color mode)
```

### Data Flow
```
t → per-band per-row: sum octave sines → curtain column
→ brightness falloff → char + color + attribute → screen
```

