# Concept: Plasma Effect

## Pass 1 — Understanding

### Core Idea
Generate a dynamic, colorful animated pattern by summing multiple sine waves over 2D coordinates plus time. The result is a smoothly varying field of colors that ripples and swirls. A classic demoscene effect.

### Mental Model
Imagine overlaying several colored water ripples from different sources. Each adds a wave pattern based on distance or position. Their sum creates interference patterns that change over time as the waves drift. The palette mapping turns amplitude into vivid cycling colors.

### Key Equations
```
v(x, y, t) = sin(x/16 + t)
           + sin(y/8 + t)
           + sin((x+y)/16 + t)
           + sin(sqrt(x²+y²)/8 + t)
```
Normalize v to [0,1], then map to a cyclic palette.

### Data Structures
- No grid allocation needed — compute each pixel on the fly
- Palette: array of (r,g,b) or color-pair indices cycling through hue
- `g_time`: accumulated float

### Non-Obvious Decisions
- **Palette cycling**: The plasma value v is offset by time: `palette_index = (int)(v*N + t*SPEED) % N`. This makes the colors flow without recomputing the field.
- **Distance term**: `sin(sqrt(x²+y²)/scale)` creates circular ripples from the center. It's expensive (sqrt per pixel) — precompute or use `hypot()`.
- **Multiple waves = richness**: A single sine wave gives a boring gradient. Three or more overlapping waves create the complex interference pattern.
- **ASCII terminal limitation**: You can only use ~16 color pairs. Map the continuous value to these pairs. Use different characters (space, `.`, `:`, `#`) to add another density dimension.

## From the Source

**Algorithm:** Analytic plasma — no state grid, no simulation. Classic demoscene technique (Commodore 64 era, ~1990s). Each frame `v(col,row,t)` is computed as a sum of 4 sine waves. The radial term `sin(√(dx²+(2·dy)²)·f4)` creates circular ripples centred on screen; the factor 2 on dy corrects for terminal cell aspect ratio (cells taller than wide).

**Math:** Normalization: `v_norm = (v + 4) / 8` — 4 unit-amplitude waves → sum ∈ [-4, 4]. Sinusoidal RGB palette: `r = 0.5 + 0.5·cos(2π(t + 0.0))`, `g = 0.5 + 0.5·cos(2π(t + 0.33))`, `b = 0.5 + 0.5·cos(2π(t + 0.67))` — evenly-spaced phase offsets produce full-spectrum hue cycle. Palette phase cycles at `CYCLE_HZ=0.20` Hz. N_PAL=14 entries per theme, N_THEMES=4, N_FREQ_PRESETS=4.

---

### Key Constants
| Name | Role |
|------|------|
| SPEED | how fast the palette cycles |
| SCALE_X/Y | spatial frequency of each wave |
| N_WAVES | number of summed sine terms |
| N_COLORS | number of palette entries |

### Open Questions
- What frequency ratio creates the most complex interference pattern?
- Replace sin with saw or square waves — how does the pattern change?
- Can you add a "zoom" parameter that moves the camera into or out of the pattern?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `THEMES[4][14]` | `PalEntry[4][14]` | ~560 B | per-theme palette: 14 character/color/attr entries |
| `FREQ_PRESETS[4]` | `FreqPreset[4]` | ~160 B | four spatial frequency and time-speed presets |
| `scene.plasma` | `Plasma` | ~16 B | active state: time accumulator, freq preset, theme, paused flag |

## Pass 2 — Implementation

### Pseudocode
```
plasma_value(col, row, t):
    x = col - cols/2.0
    y = row - rows/2.0
    v  = sin(x/16.0 + t)
    v += sin(y/8.0  + t*1.3)
    v += sin((x+y)/16.0 + t*0.7)
    v += sin(sqrt(x*x + y*y)/8.0)
    return (v + 4) / 8.0     # normalize to [0,1]

palette_color(v, t):
    hue = fmod(v + t*SPEED, 1.0)
    return hue_to_color_pair(hue)

density_char(v):
    chars = " .:-=+*#@█"
    return chars[(int)(v * (len-1))]

draw(t):
    for row in 0..rows:
        for col in 0..cols:
            v = plasma_value(col, row, t)
            cp = palette_color(v, t)
            ch = density_char(v)
            attron(COLOR_PAIR(cp))
            mvaddch(row, col, ch)
```

### Module Map
```
§1 config    — SPEED, wave scale constants, N_COLORS
§2 palette   — hue_to_color_pair(), build 256-color gradient
§3 plasma    — plasma_value() combining sine terms
§4 draw      — sweep all pixels per frame
§5 app       — main loop, keys (speed, wave params, pause)
```

### Data Flow
```
t → per-pixel plasma_value(col,row,t) → [0,1]
→ palette_color (hue cycling) + density_char → screen
```

