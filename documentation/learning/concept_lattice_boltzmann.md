# Concept: Lattice Boltzmann Method (D2Q9)

## Pass 1 — Understanding

### Core Idea
Simulate viscous 2-D fluid flow by evolving particle distribution functions on a fixed lattice. Each cell holds 9 "partial densities" — how many particles are moving in each of 9 directions. Each step: (1) relax toward equilibrium (collision), (2) propagate to neighbours (streaming). Navier-Stokes macroscopic behavior emerges automatically from these mesoscopic rules.

### Mental Model
Imagine 9 lanes of traffic at every intersection. Most traffic stays put (center lane, weight 4/9), some goes in cardinal directions (weight 1/9), a little goes diagonally (weight 1/36). Every tick, each intersection decides how much traffic to redirect toward equilibrium, then all traffic moves to adjacent intersections. The result: fluid-like flow with vortices, boundary layers, and wakes — without ever solving a pressure equation.

### Key Equations
```
BGK collision:
f_i*(x,t) = f_i − ω·(f_i − f_eq_i)

Maxwell-Boltzmann equilibrium:
f_eq_i = w_i · ρ · [1 + (e_i·u)/cs² + (e_i·u)²/(2cs⁴) − u²/(2cs²)]

Streaming:
f_i(x + e_i·dt, t+dt) = f_i*(x, t)

Macroscopic moments:
ρ = Σᵢ f_i
ρ·u = Σᵢ f_i · e_i
```

### Data Structures
- `f[ROWS][COLS][9]`: distribution functions (9 per cell)
- `ftmp[ROWS][COLS][9]`: double-buffer for streaming
- `rho[ROWS][COLS]`, `ux[ROWS][COLS]`, `uy[ROWS][COLS]`: macroscopic fields
- `solid[ROWS][COLS]`: bool mask for obstacles

### Non-Obvious Decisions
- **Double buffer for streaming**: Streaming reads neighbours; if done in-place, an already-streamed cell overwrites a value still needed. Double-buffer: collide into ftmp, stream from ftmp into f.
- **cs²=1/3**: This is a lattice artifact — it is the second moment of the D2Q9 weight distribution, not a tunable parameter.
- **τ > 0.5**: Ensures ν > 0. τ < 0.5 would give negative viscosity — nonphysical and immediately unstable.
- **Lattice Mach number Ma < 0.3**: The feq is a quadratic Taylor expansion of the full M-B distribution. Higher-order terms are negligible only for Ma << 1. Ma > 0.3 introduces significant compressibility errors.
- **Bounce-back at half-cell**: The effective no-slip plane sits halfway between the fluid cell and the solid cell, not exactly at the cell boundary. For low Re this is acceptable; for high Re accuracy, link-wise bounce-back or interpolated BC are used.

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| TAU | 0.65 | relaxation time → ν=(τ−0.5)/3 |
| U_IN | 0.1 | inlet velocity (lattice units) |
| Re | U·D/(2ν) | Reynolds number (D = cylinder diameter) |
| cs² | 1/3 | lattice speed of sound squared (fixed) |

### Open Questions
- Why does the stability requirement τ > 0.5 arise from ν = (τ−0.5)·cs²?
- What is the Strouhal number and how does it relate to shedding frequency?
- Why does the outlet zero-gradient BC prevent wave reflection?

## From the Source

**Algorithm:** D2Q9 LBM with BGK collision, double-buffer streaming, bounce-back no-slip, inlet equilibrium, outlet zero-gradient BC. Single-cylinder obstacle.

**Physics/References:** Chapman-Enskog expansion recovers N-S to second order in Ma. D2Q9 lattice: Qian, d'Humières & Lallemand (1992). BGK: Bhatnagar, Gross & Krook (1954). Kármán vortex street Re>47: von Kármán (1912), confirmed by LBM in Mei, Shyy & Luo (2000).

**Math:** Viscosity-relaxation: ν = (τ−0.5)·cs² = (τ−0.5)/3. For TAU=0.65: ν=0.05/3≈0.0167. Re = U·D/ν. For U=0.1, D=10 cells, ν=0.0167: Re≈60 → just above shedding threshold.

**Performance:** O(ROWS·COLS·9) per step: 9 feq + BGK + stream per cell. For 200×80 grid: 144k operations/step. Memory: 2 × 200 × 80 × 9 × 4B = ~576 KB.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `f[R][C][9]` | `float[R×C×9]` | 9× grid | distribution functions |
| `ftmp[R][C][9]` | `float[R×C×9]` | 9× grid | streaming double-buffer |
| `rho[R][C]` | `float[R×C]` | 1× grid | macroscopic density |
| `ux[R][C]`, `uy[R][C]` | `float[R×C]` | 1× each | macroscopic velocity |
| `solid[R][C]` | `bool[R×C]` | 1× grid | obstacle mask |

---

## Pass 2 — Implementation

### Pseudocode
```
lbm_step():
    # collision: relax toward equilibrium
    for i,j:
        compute_macroscopic(f[i][j], &rho, &ux, &uy)
        for k in 0..8:
            feq = w[k]*rho*(1 + dot(e[k],u)/cs2
                           + dot(e[k],u)²/(2*cs2²) - |u|²/(2*cs2))
            ftmp[i][j][k] = f[i][j][k] - omega*(f[i][j][k] - feq)

    # streaming: propagate to neighbours
    for i,j, k in 0..8:
        ni = i + e[k][1];  nj = j + e[k][0]
        if in_bounds(ni, nj):
            f[ni][nj][k] = ftmp[i][j][k]

    # bounce-back no-slip at solid cells
    for i,j where solid[i][j]:
        for k in 0..8:
            f[i][j][k] = ftmp[i][j][opposite[k]]

    # inlet BC: equilibrium at u=U_IN
    for i in col 0: f[i][0] = feq(rho_ref, U_IN, 0)

    # outlet BC: zero gradient (copy from second-to-last column)
    for i in col COLS-1: f[i][COLS-1] = f[i][COLS-2]
```

### Module Map
```
§1 config        — TAU, U_IN, CYL_R, TAU_MIN, lattice weights/vectors
§2 feq           — maxwell_boltzmann_eq() with Taylor expansion
§3 macroscopic   — compute_moments() ρ=Σfi, ρu=Σfi·ei
§4 collide       — BGK f_i* = f − ω(f−feq) into ftmp
§5 stream        — propagate ftmp→f + pointer swap
§6 BC            — bounce_back(), inlet_bc(), outlet_bc()
§7 vorticity     — finite-diff ωz = ∂uy/∂x − ∂ux/∂y
§8 render        — velocity magnitude / vorticity → color + arrows
§9 app           — main loop, Re/tau controls, obstacle placement
```

### Data Flow
```
f[i][j] → compute_macroscopic → rho, ux, uy
rho, ux, uy → feq → BGK collision → ftmp
ftmp → streaming → f (next step)
f at boundaries → bounce-back / inlet / outlet
ux, uy → vorticity → colormap → screen
```
