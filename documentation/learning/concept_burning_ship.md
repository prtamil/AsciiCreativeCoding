# Concept: Burning Ship Fractal

## Pass 1 — Understanding

### Core Idea
A variant of the Mandelbrot set with one twist: before squaring, take the absolute value of both real and imaginary parts. `z → (|Re(z)| + i|Im(z)|)² + c`. The resulting fractal has a striking resemblance to a burning ship when viewed upside-down.

### Mental Model
The Mandelbrot set folds the complex plane smoothly. The burning ship folds it by first reflecting both axes into the positive quadrant (the absolute value). This reflection breaks the left-right symmetry and creates the jagged, asymmetric ship shape.

### Key Equations
```
z₀ = 0
z_{n+1} = (|Re(z_n)| + i·|Im(z_n)|)² + c
```
Expanded:
```
x_{n+1} = x_n² - y_n² + Re(c)
y_{n+1} = 2·|x_n|·|y_n| + Im(c)    ← the abs() trick
```
Color by escape time (how many iterations before |z|>2).

### Data Structures
- No grid needed — compute each pixel independently
- Each pixel maps to a complex number c
- Escape iteration count → color

### Non-Obvious Decisions
- **View upside-down**: The "burning ship" shape appears when the imaginary axis is flipped. Negate y when displaying: `Im(c) = -(screen_y - center_y)*scale`.
- **Smooth coloring**: `iter + 1 - log(log|z|)/log(2)` gives smooth banding without hard color steps.
- **Zooming into the hull**: The most detailed areas are near the main ship body and the smaller ship-shaped mini-copies visible on the mast and hull.
- **Same escape-time structure as Mandelbrot**: Just two abs() calls added. The rest of the code is identical.

### Key Constants
| Name | Role |
|------|------|
| MAX_ITER | 100–500 for exploration |
| BAILOUT | 2.0 (standard escape radius) |
| center_x, center_y | view center in complex plane |
| scale | zoom level (pixel → complex plane units) |

### Open Questions
- Find the "ship" by looking at the region Re(c)∈[-2,2], Im(c)∈[-2,1] upside-down.
- Compare side-by-side with Mandelbrot at the same zoom level.
- Where are the "mini-ships" (self-similar copies)?

---

## Pass 2 — Implementation

### Pseudocode
```
render():
    for each pixel (px, py):
        cx = center_x + (px - cols/2) * scale
        cy = -(center_y + (py - rows/2) * scale)  # flip y for ship orientation
        x=0, y=0
        for iter in 0..MAX_ITER:
            if x²+y² > 4: break    # escaped
            x2 = x*x - y*y + cx
            y2 = 2*abs(x)*abs(y) + cy    # the abs() trick
            x=x2; y=y2
        color = escape_color(iter, x, y)
        draw_char(px, py, color)

escape_color(iter, x, y) → char, color_pair:
    if iter == MAX_ITER: return ' ', BLACK    # inside
    # smooth: add fractional part
    smooth = iter + 1 - log(log(sqrt(x²+y²)))/log(2)
    hue = smooth / MAX_ITER
    return density_char(hue), hue_to_color(hue)
```

### Module Map
```
§1 config    — MAX_ITER, BAILOUT, initial center/scale
§2 fractal   — burning_ship_iter(), smooth_color()
§3 draw      — render() pixel sweep
§4 app       — main loop, keys (zoom in/out, pan, reset, compare mode)
```

### Data Flow
```
pixel (px,py) → complex c → iterate (abs trick) → escape iter
→ smooth color → terminal char + color pair → screen
```

## From the Source

**Algorithm:** Escape-time iteration — identical to Mandelbrot except for the absolute-value fold applied before each squaring. This "folding" maps the complex plane to the first quadrant before squaring, breaking the 4-fold symmetry of Mandelbrot and creating the distinctive ship/flame asymmetry.

**Math:** Iteration: `z ← (|Re(z)| + i|Im(z)|)² + c`. Expanding: `Re_new = Re(z)² − Im(z)² + Re(c)`, `Im_new = 2|Re(z)|·|Im(z)| + Im(c)`. The `|Im|` term forces the imaginary component positive after each step, creating downward-pointing flames rather than Mandelbrot's symmetric bulbs. Escape condition: `|z|² > 4` (equivalent to `|z| > 2`).

**Performance:** Smooth colouring via fractional escape count: `t = iter + 1 − log₂(log₂|z|)`. This removes the "banding" of integer escape counts and produces smooth colour gradients across the boundary. Coordinate note: `burning_ship_iter()` negates ci before iterating to flip the image vertically so the ship appears hull-down.
