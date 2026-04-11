# Concept: Newton Fractal

## Pass 1 — Understanding

### Core Idea
Apply Newton-Raphson root-finding to a complex polynomial. Starting from each pixel's complex number z₀, iterate z ← z - f(z)/f'(z) until convergence to a root. Color by which root it converged to, shade by how many iterations it took. The boundaries between basins of attraction are infinitely fractal.

### Mental Model
Imagine a mountainous landscape with three valleys (the three roots of z³-1). You're a ball rolling downhill. Most starting points roll into the nearest valley quickly, but near the ridge-lines between valleys, tiny changes in starting position send you to completely different valleys. Those ridge-lines are the fractal.

### Key Equations
Newton iteration for polynomial f(z):
```
z_{n+1} = z_n - f(z_n) / f'(z_n)
```

For f(z) = z³ - 1:
```
f(z)  = z³ - 1
f'(z) = 3z²
z_{n+1} = z - (z³-1)/(3z²) = (2z³+1)/(3z²)
```

Roots of z³-1 = (1, e^{2πi/3}, e^{4πi/3}).

### Data Structures
- No grid needed — compute each pixel independently
- `complex float` arithmetic (or manual re/im)
- Color lookup: 3 roots → 3 colors, iteration count → brightness

### Non-Obvious Decisions
- **Damped Newton**: Multiply the step by a factor α∈(0,1) to slow convergence and reveal more fractal detail: `z -= α * f(z)/f'(z)`.
- **Generalize to any polynomial**: Store roots as an array, compute f(z) = Π(z-rootᵢ), f'(z) by product rule or numerical differentiation.
- **Convergence test**: Stop when |f(z)| < ε or |Δz| < ε. Maximum iterations ~50–100.
- **Color by nearest root not just convergence**: After stopping, find which root z is closest to. This reveals the basin of attraction structure.
- **Zoom**: Newton fractals have infinite detail at the basin boundaries — implement zoom.

### Key Constants
| Name | Role |
|------|------|
| MAX_ITER | maximum Newton iterations (50–100) |
| TOL | convergence tolerance (1e-6) |
| α | Newton step damping (1.0 = standard, 0.5–0.8 = slowed) |
| ZOOM | current view scale |

### Open Questions
- Try f(z) = z⁴-1 (four roots). What changes?
- What happens with α < 0 (anti-Newton)?
- Can you color by iteration count within each basin for smooth shading?

---

## Pass 2 — Implementation

### Pseudocode
```
complex roots[3] = {1, e^(2πi/3), e^(4πi/3)}

newton_iterate(z0) → (root_index, iterations):
    z = z0
    for n in 0..MAX_ITER:
        fz  = z³ - 1
        fpz = 3*z²
        if |fpz| < 1e-10: break   # near critical point
        z -= ALPHA * fz/fpz
        if |fz| < TOL:
            return nearest_root(z, roots), n
    return nearest_root(z, roots), MAX_ITER

render():
    for each pixel (px, py):
        z0 = screen_to_complex(px, py, zoom, center)
        (root_idx, iters) = newton_iterate(z0)
        brightness = 1.0 - iters/MAX_ITER
        color = BASE_COLOR[root_idx] * brightness
        draw_char(px, py, density_char(brightness), color_pair[root_idx])
```

### Module Map
```
§1 config    — MAX_ITER, TOL, ALPHA, initial zoom/center
§2 complex   — complex multiply/divide (or use <complex.h>)
§3 fractal   — newton_iterate(), nearest_root()
§4 draw      — render() parallelizable row-by-row, color mapping
§5 app       — main loop, keys (zoom, pan, alpha, root count)
```

### Data Flow
```
pixel (px,py) → screen_to_complex → newton_iterate
→ (root_index, iters) → color + brightness → terminal char
```
