# Concept: Reaction Wave (FitzHugh-Nagumo / Excitable Media)

## Pass 1 — Understanding

### Core Idea
Model excitable media — systems where cells can be excited, propagate a wave, then enter a refractory period before resetting. Models action potentials in nerves, cardiac waves, spiral waves in chemical reactions (Belousov-Zhabotinsky reaction).

### Mental Model
Imagine a field of dominoes. Knock one over and it knocks down its neighbors, which knock down theirs. After falling, each domino stands back up slowly. By the time neighbors' waves arrive, fallen dominoes might be standing again. This creates traveling waves and spiral patterns.

### Key Equations
FitzHugh-Nagumo model (simplified excitable cell):
```
du/dt = u - u³/3 - v + I_ext + D∇²u    ← fast activator
dv/dt = ε(u + a - b·v)                 ← slow inhibitor
```
Or simpler two-variable Barkley model:
```
du/dt = (1/ε)·u(1-u)(u - (v+b)/a) + D∇²u
dv/dt = u - v
```

### Data Structures
- `u[H][W]`, `v[H][W]`: activator and inhibitor fields
- Double buffer for each field
- Laplacian computed with 5-point stencil

### Non-Obvious Decisions
- **Two variables needed**: A single variable can't produce refractory behavior. The inhibitor v is what prevents re-excitation — it rises after u spikes and takes time to decay.
- **Laplacian for spatial coupling**: `∇²u ≈ u[i+1,j]+u[i-1,j]+u[i,j+1]+u[i,j-1] - 4·u[i,j]`. This is what spreads the excitation to neighboring cells.
- **Spiral waves need broken wavefront**: Start by initiating a planar wave, then block part of it (refractory region). The broken end curls into a spiral.
- **Explicit Euler works here**: Unlike fluid simulations, explicit Euler is stable for this system with small enough dt (CFL condition on diffusion: dt < dx²/(4D)).
- **Color by (u,v) phase**: Map activator u to brightness and inhibitor v to color. Shows the excitation/recovery cycle visually.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| D | 0.1–1.0 | diffusion coefficient |
| ε | 0.1–0.02 | timescale ratio (fast activator / slow inhibitor) |
| a, b | ~1.1, 0.5 | Barkley parameters |
| dt | 0.01–0.1 | timestep |

### Open Questions
- What initial conditions produce a rotating spiral wave?
- What happens when you increase D? (faster wave propagation)
- Can two spiral waves annihilate each other?

## From the Source

**Algorithm:** Explicit (Forward) Euler integration of a 2-variable PDE. Each tick: compute Laplacian ∇²u with a 5-point stencil, evaluate reaction terms, update u and v from du/dt and dv/dt. Unlike Gray-Scott (autocatalytic), FitzHugh-Nagumo has a single nonlinear term (u³) making it analytically tractable.

**Physics/References:** FitzHugh-Nagumo equations (1961/1962). Models cardiac action potentials (Hodgkin-Huxley simplified). u = membrane voltage (fast activator); v = recovery variable (slow inhibitor, restores resting state). Fixed point: u* ≈ −1.2, v* ≈ −0.625 (resting state). A stimulus above threshold triggers a full action potential; the refractory period (v recovery) prevents back-propagation — this is why heart-muscle waves travel as rings, not balls. FN_EPS=0.08 sets the v/u time-scale ratio; small ε → slow inhibitor → longer action potential duration.

**Math:** CFL stability: FN_DT · FN_D / dx² < 0.25.

**Performance:** STEPS_PER_FRAME=8 sub-steps maintain CFL stability while displaying at 30 fps. Without sub-stepping, a single larger dt would violate CFL and produce NaN or blow-up. O(W×H) per step with a fixed 5-point Laplacian stencil.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_u[GRID_H_MAX][GRID_W_MAX]` | `float[100][320]` | ~125 KB | activator u (membrane voltage analogue) |
| `g_v[GRID_H_MAX][GRID_W_MAX]` | `float[100][320]` | ~125 KB | inhibitor v (recovery variable) |
| `g_u2[GRID_H_MAX][GRID_W_MAX]` | `float[100][320]` | ~125 KB | scratch buffer for next u |
| `g_v2[GRID_H_MAX][GRID_W_MAX]` | `float[100][320]` | ~125 KB | scratch buffer for next v |

---

## Pass 2 — Implementation

### Pseudocode
```
laplacian(f, i, j):
    return f[i+1][j] + f[i-1][j] + f[i][j+1] + f[i][j-1] - 4*f[i][j]

step():
    for i,j:
        Lu = laplacian(u, i, j)
        Lv = laplacian(v, i, j)   # optional: if v diffuses
        # FitzHugh-Nagumo
        du = u[i][j] - u[i][j]³/3 - v[i][j] + D*Lu
        dv = EPS * (u[i][j] + A - B*v[i][j])
        u_new[i][j] = u[i][j] + dt*du
        v_new[i][j] = v[i][j] + dt*dv
    swap(u, u_new); swap(v, v_new)

init_spiral():
    # horizontal planar wave
    for i in 0..H: u[i][W/4] = 1.0
    # block lower half to break wavefront
    for i in H/2..H: v[i][W/4] = 1.0   # refractory

draw():
    for i,j:
        # map u ∈ [-1,2] to [0,1]
        brightness = (u[i][j]+1) / 3
        char = density_char(brightness)
        color = hue_to_pair(v[i][j])
        attron(COLOR_PAIR(color))
        mvaddch(i+offset, j+offset, char)
```

### Module Map
```
§1 config    — H, W, D, EPS, A, B, DT
§2 init      — planar wave, spiral IC, random perturbation
§3 physics   — laplacian(), step()
§4 draw      — (u,v) → color + char
§5 app       — main loop, keys (init mode, speed, param sliders)
```

### Data Flow
```
(u,v) fields → laplacian coupling → FitzHugh-Nagumo ODE → new fields
u → brightness; v → color → ASCII display
```
