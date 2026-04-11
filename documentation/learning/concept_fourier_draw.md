# Concept: Fourier Drawing (Epicycles)

## Pass 1 — Understanding

### Core Idea
Any closed curve can be decomposed into a sum of rotating circles (epicycles) via the Discrete Fourier Transform. Given N sample points of a shape, the DFT gives N complex coefficients, each representing a circle with a specific radius (amplitude), rotation speed (frequency), and starting angle (phase). Drawing them in sequence reconstructs the original shape.

### Mental Model
Each Fourier coefficient is a spinning arm. The first arm (DC component) is the center. The second arm spins at frequency 1, the third at frequency 2 (or -1). Each arm's tip traces the next arm's pivot. The last arm's tip traces the original curve. Like adding harmonics to a fundamental frequency.

### Key Equations
DFT of N complex points z[k] = x[k] + i·y[k]:
```
Z[n] = Σ_{k=0}^{N-1} z[k] · e^{-2πi·k·n/N}
```

Reconstruction at time t ∈ [0,1]:
```
z(t) = Σ_n (Z[n]/N) · e^{2πi·n·t}
```

Each term `Z[n]/N · e^{2πi·n·t}` is a rotating circle with:
- radius = |Z[n]|/N
- angular velocity = 2π·n rad/period
- initial phase = arg(Z[n])

### Data Structures
- Input: N sample points (x,y) from a drawing or preset shape
- `coeff[N]`: complex DFT coefficients, sorted by decreasing magnitude
- Epicycle state: array of (radius, freq, phase) — one per coefficient
- Trail buffer: last M positions of the final arm tip

### Non-Obvious Decisions
- **Sort by magnitude**: Draw largest circles first, smallest last. This gives the best visual and converges the shape quickly as you add more circles.
- **Animate adding circles**: Show the drawing first with N=1 circle (rough), then N=2, N=4... up to the full set. Reveals how Fourier decomposition works.
- **Input via file or preset**: Users can provide arbitrary closed curves. Or use preset shapes: square, star, letter, Fourier approximation of a circle.
- **Complex DFT**: Treat each (x,y) point as a complex number x+iy. The DFT output contains both positive and negative frequencies — both are needed to reconstruct asymmetric shapes.
- **Trail**: Draw the path traced by the last arm tip as a line. This is the reconstructed shape gradually appearing.

### Key Constants
| Name | Role |
|------|------|
| N | number of sample points from the input shape |
| N_CIRCLES | how many epicycles to show (1..N) |
| TRAIL_LEN | history of tip positions to draw |
| SPEED | animation speed (t increment per frame) |

### Open Questions
- How many circles are needed to approximate a square to within 1%?
- Why are negative frequencies needed for non-symmetric shapes?
- Try drawing your name with mouse input, then replay with epicycles

---

## Pass 2 — Implementation

### Pseudocode
```
# Input: points[N] = (x,y) as complex float
dft(points) → coeffs[N]:
    for n in 0..N:
        Z = 0
        for k in 0..N:
            angle = -2*PI*k*n/N
            Z += points[k] * exp(i*angle)
        coeffs[n] = Z / N

# Sort by magnitude descending
sort(coeffs, key=|c|, reverse=True)

draw_frame(coeffs, t, n_circles, trail):
    x=center_x, y=center_y
    for k in 0..n_circles:
        freq = coeffs[k].freq
        angle = 2*PI*freq*t + coeffs[k].phase
        r = coeffs[k].magnitude
        # draw circle outline
        draw_circle(x, y, r)
        # advance arm
        x += r*cos(angle)
        y += r*sin(angle)
        # draw arm
        draw_line(prev_x,prev_y, x,y)
    trail.push(x, y)
    draw_trail(trail)
```

### Module Map
```
§1 config    — N, N_CIRCLES, TRAIL_LEN, SPEED
§2 dft       — dft(), sort_by_magnitude()
§3 draw      — draw_circle(), draw_arm(), draw_trail()
§4 animate   — advance t, update trail, render frame
§5 app       — main, keys (speed, circle count, preset shape)
```

### Data Flow
```
input points → DFT → sorted coefficients
→ per frame: animate t → reconstruct position → trail
→ draw epicycles + arms + trail → screen
```
