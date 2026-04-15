# Concept: Lenia (Continuous Cellular Automaton)

## Pass 1 — Understanding

### Core Idea
Lenia generalizes Conway's Game of Life to continuous space, time, and states. Instead of binary cells and a Moore neighborhood, each cell has a real value [0,1] and the neighborhood is a smooth kernel (bell-shaped ring). The update is: convolve with kernel, apply growth function, step state.

### Mental Model
Conway's GoL uses a counting rule: "3 neighbors → birth, 2–3 neighbors → survive." Lenia replaces that with: measure weighted neighbor average, check if it's in the "life zone" of a bell curve, add or subtract from current value. The result is living creatures that move, divide, and interact like microorganisms.

### Key Equations
```
U(x,t) = K ⊛ A(·,t)                 ← convolution
G(u)   = 2·exp(-((u-μ)/(σ))²) - 1  ← growth function (bell)
A(x,t+dt) = clamp(A(x,t) + dt·G(U(x,t)), 0, 1)
```

Where K is a ring-shaped kernel (exponential shell) and μ,σ parameterize the growth function.

### Data Structures
- `grid[H][W]`: float array, values in [0,1]
- `kernel[KH][KW]`: precomputed convolution kernel
- Double buffer: swap after each step

### Non-Obvious Decisions
- **Ring kernel, not disc**: The kernel is nonzero only in an annular ring (peak at radius R, zero inside and outside). This creates the "sensing neighborhood" of a cell.
- **Toroidal boundaries**: Wrap edges to avoid boundary artifacts in the convolution.
- **FFT convolution**: For large grids, the O(N²·K²) direct convolution is too slow. Use FFT: O(N²·logN). For small grids (64×64 with small kernel), direct convolution is fine.
- **Growth function symmetry**: The bell shape means "life zone" is centered at μ. Too few neighbors (u<μ) or too many (u>μ) both lead to decay.
- **dt << 1**: Unlike GoL which is discrete, Lenia advances in small steps. Too large dt makes it unstable.

### Key Constants
| Name | Role |
|------|------|
| R | kernel radius (neighborhood size) |
| T | timescale (1/dt) |
| μ | center of growth bell curve |
| σ | width of growth bell curve |
| dt | time step (typically 0.1) |

### Open Questions
- What happens when you change μ? (different "preferred density")
- Can you find parameters that produce orbium (the moving creature)?
- How does periodic boundary condition differ from zero-padding?

## From the Source

**Algorithm:** Continuous cellular automaton with convolution kernel. Each step: convolve state grid with kernel K (weighted ring), evaluate growth function G, apply update rule. Naïve O(W·H·πR²) convolution — for R=13 and a 200×60 grid that's ~6M ops per step.

**Math:** Growth function G(x) = exp(−(x−μ)²/(2σ²)) — a Gaussian centred at μ. When the kernel convolution result matches μ exactly, G=1 and the cell grows toward 1. Far from μ, G→0 and the cell decays. The parameter pair (μ, σ) defines a "species" of creature. The ring kernel captures "neighbourhood density at radius R" — analogous to how a biological cell senses chemical gradients.

**Physics/References:** Lenia (Bert Wang-Chak Chan, 2019). A continuous generalisation of Conway's Game of Life. Self-organisation emerges from the tension between growth (G>0.5) and decay (G<0.5).

**Performance:** Pre-building the kernel O(R²) once amortises cost across all W×H cells per step. Normalised so total kernel weight = 1. Preset creatures: Orbium (μ=0.15, σ=0.015, R=13), Aquarium (μ=0.26, σ=0.036, R=10), Scutium (μ=0.17, σ=0.015, R=8).

---

## Pass 2 — Implementation

### Pseudocode
```
build_kernel(R):
    for each offset (di,dj) within radius R:
        r = sqrt(di²+dj²) / R
        if 0 < r < 1:
            K[di][dj] = exp(4 - 4/(r*(1-r)))  # bump function
    normalize K so sum = 1

convolve(grid, kernel, R) → u[H][W]:
    for each cell (i,j):
        u[i,j] = 0
        for each kernel offset (di,dj):
            ni = (i+di) mod H
            nj = (j+dj) mod W
            u[i,j] += K[di][dj] * grid[ni][nj]

growth(u) → g:
    return 2*exp(-((u-MU)/SIGMA)²) - 1

step():
    compute u = convolve(grid, kernel, R)
    for each cell:
        grid_new[i][j] = clamp(grid[i][j] + DT*growth(u[i][j]), 0, 1)
    swap(grid, grid_new)
```

### Module Map
```
§1 config    — W, H, R, MU, SIGMA, DT
§2 kernel    — build_kernel(), normalize
§3 physics   — convolve(), growth(), step()
§4 draw      — value [0,1] → ASCII density chars → color
§5 app       — main loop, keys (param sliders, reset, presets)
```

### Data Flow
```
grid[H][W] → convolve with K → u[H][W] → growth(u) → new grid
grid values [0,1] → map to " .:-=+*#@" or color gradient → screen
```
