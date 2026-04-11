# Concept: Perlin Landscape (fBm Terrain + Parallax)

## Pass 1 — Understanding

### Core Idea
Generate a landscape by summing multiple octaves of Perlin noise (Fractional Brownian Motion). Display it as a scrolling side-view with parallax layers — distant mountains move slower than foreground hills, giving the illusion of depth.

### Mental Model
Look out the window of a moving train. Trees close by blur past quickly. Hills in the middle distance move slowly. Mountains on the horizon barely move. Parallax is that differential scrolling speed. Each layer is a Perlin noise profile with different frequencies — high-frequency for bumpy foreground, low-frequency for smooth background mountains.

### Key Equations
Perlin noise (1D, for landscape profile):
```
noise(x) = fade(frac(x)) interpolated from lattice gradients
```

fBm (Fractional Brownian Motion):
```
fBm(x) = Σ_{k=0}^{octaves-1} amplitude^k · noise(frequency^k · x)
```

Typical values: amplitude=0.5, frequency=2.0 (lacunarity).

Parallax scrolling: layer k scrolls at `speed_k = base_speed / (k+1)`.

### Data Structures
- N layers, each with: `offset_x`, `speed`, `amplitude_scale`, `color_pair`
- For each layer: compute noise profile of height for each column
- Screen: draw filled columns from noise height down to ground

### Non-Obvious Decisions
- **Perlin vs. value noise**: Perlin noise has continuous gradients — smooth hills. Value noise (interpolated random values) is blockier. Use Perlin for natural landscapes.
- **Layered rendering**: Draw from back to front. Background (slowest, lightest) first, foreground (fastest, darkest) last.
- **Color by layer**: Background mountains are light gray/blue (atmospheric haze). Mid-ground is green. Foreground is dark green/brown.
- **Wrapping**: Noise should tile seamlessly as the camera scrolls. Use `noise(fmod(x, period))` with a large period.
- **Amplitude scale by layer**: Background has smaller height variation (smoother silhouette). Foreground has larger variation (rocky terrain).

### Key Constants
| Name | Role |
|------|------|
| N_LAYERS | number of parallax layers (3–5) |
| OCTAVES | fBm octaves per layer (4–8) |
| BASE_SPEED | scrolling speed of foreground layer |
| SEA_LEVEL | row below which is water (blue fill) |

### Open Questions
- How does the Hurst exponent (persistence) relate to the smoothness of the terrain?
- Can you add clouds (separate Perlin layer at the top)?
- What frequency ratio (lacunarity) produces the most natural-looking terrain?

---

## Pass 2 — Implementation

### Pseudocode
```
perlin_1d(x) → float in [-1,1]:
    ix = floor(x)
    fx = x - ix
    fade_t = fx*fx*fx*(fx*(fx*6-15)+10)   # smoothstep
    g0 = gradient(hash(ix))    # -1 or +1
    g1 = gradient(hash(ix+1))
    return lerp(g0*fx, g1*(fx-1), fade_t)

fbm(x, octaves) → float:
    val=0, amp=1, freq=1, max_val=0
    for k in 0..octaves:
        val += amp * perlin_1d(x * freq)
        max_val += amp
        amp *= PERSISTENCE; freq *= LACUNARITY
    return val / max_val

draw_layer(layer, offset_x):
    for col in 0..cols:
        x = (col + offset_x) * FREQ_SCALE
        h = fbm(x, OCTAVES) * layer.amp_scale
        row_start = (int)((1 - (h+1)/2) * rows)   # map [-1,1] to row
        attron(COLOR_PAIR(layer.color))
        for row in row_start..rows:
            mvaddch(row, col, layer.fill_char)

main_loop():
    for each frame:
        t += DT
        erase()
        for layer from back to front:
            layer.offset_x += layer.speed * DT
            draw_layer(layer)
        draw_HUD()
        refresh
```

### Module Map
```
§1 config    — N_LAYERS, layer params (speed, color, amp, freq)
§2 noise     — hash(), perlin_1d(), fbm()
§3 draw      — draw_layer(), back-to-front order
§4 app       — main loop, keys (speed, pause, layer toggle)
```

### Data Flow
```
t → per-layer offset_x → per-column fbm(x) → height profile
→ fill column from height down → back-to-front draw → screen
```
