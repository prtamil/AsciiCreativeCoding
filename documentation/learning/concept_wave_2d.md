# Concept: 2D Wave Equation

## Pass 1 — Understanding

### Core Idea
Simulate the 2D wave equation — ripples on a water surface, sound in a room, light in a waveguide. Each point oscillates and couples to its neighbors. FDTD (finite difference time domain) discretizes both space and time.

### Mental Model
Drop a stone in a pond. Ripples spread outward as circles. When they hit the wall they reflect. Two stones create interference: where crests meet, you get a double crest; where a crest meets a trough, they cancel. This is Huygens' principle.

### Key Equations
2D wave equation:
```
∂²u/∂t² = c² · (∂²u/∂x² + ∂²u/∂y²)
```

FDTD discretization:
```
u[t+1][i][j] = 2·u[t][i][j] - u[t-1][i][j]
             + (c·dt/dx)² · (u[t][i+1][j] + u[t][i-1][j]
                            + u[t][i][j+1] + u[t][i][j-1]
                            - 4·u[t][i][j])
```

CFL stability condition: `c·dt/dx ≤ 1/√2` (2D).

### Data Structures
- Three frames: `u_prev[H][W]`, `u_curr[H][W]`, `u_next[H][W]`
- Absorbing boundary or reflecting boundary (Dirichlet: u=0 at edges)
- Source: point or line excitation

### Non-Obvious Decisions
- **Three time levels needed**: The second-order time derivative needs t-1, t, and t+1. Store three frames, cycle them each step.
- **CFL condition**: If c·dt/dx > 1/√2, the scheme is unstable and explodes. Check before running.
- **Absorbing vs. reflecting boundaries**: Dirichlet boundary (u=0 at edges) creates reflections. Perfectly matched layer (PML) absorbs outgoing waves — complex to implement.
- **Damping term**: Add `- damping·(u[t]-u[t-1])` to the update to model energy loss and prevent endless ringing.
- **Multiple sources**: Multiple simultaneous point sources create beautiful interference patterns (Young's double-slit, diffraction gratings).

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| C | 0.5 | wave speed (must satisfy CFL) |
| DX | 1.0 | grid spacing |
| DT | DX/(C·√2·1.1) | timestep (CFL budget) |
| DAMP | 0.001 | energy damping per step |

### Open Questions
- Add a slit barrier — can you demonstrate diffraction?
- Two point sources separated by λ — where are the interference maxima?
- How does the Huygens' principle explain why waves wrap around corners?

---

## Pass 2 — Implementation

### Pseudocode
```
step():
    for i in 1..H-1:
        for j in 1..W-1:
            laplacian = (u_curr[i+1][j] + u_curr[i-1][j]
                       + u_curr[i][j+1] + u_curr[i][j-1]
                       - 4*u_curr[i][j])
            u_next[i][j] = (2*u_curr[i][j] - u_prev[i][j]
                          + C²*laplacian
                          - DAMP*(u_curr[i][j] - u_prev[i][j]))
    # reflecting boundary (u=0)
    zero_edges(u_next)
    cycle(u_prev, u_curr, u_next)

add_source(row, col, t):
    u_curr[row][col] += AMPLITUDE * sin(2*PI*FREQ*t)

draw():
    for i,j:
        v = u_curr[i][j]   # in range [-1,1] approx
        brightness = (v + 1) / 2   # map to [0,1]
        char = wave_char(v)
        color = blue_if_negative_red_if_positive(v)
        mvaddch(i, j, char)
```

### Module Map
```
§1 config    — H, W, C, DAMP, source positions
§2 init      — zero fields, optional initial pulse
§3 physics   — step(), add_source()
§4 draw      — u value → char + color (pos/neg distinction)
§5 app       — main loop, keys (add source, barrier, speed, reset)
```

### Data Flow
```
u_prev, u_curr → FDTD update → u_next
sources injected → cycle buffers → draw u_curr as surface map
```
