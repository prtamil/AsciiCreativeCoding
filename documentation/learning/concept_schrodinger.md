# Concept: Schrödinger Equation (Quantum Mechanics)

## Pass 1 — Understanding

### Core Idea
Simulate a quantum particle in 1D using the time-dependent Schrödinger equation. The wavefunction ψ(x,t) is complex-valued; |ψ|² gives the probability density. The Crank-Nicolson scheme is used for stable, norm-preserving time evolution.

### Mental Model
A quantum particle doesn't have a definite position. Instead it has a wavefunction — a probability wave. Start with a Gaussian wave packet (like a blob of probability). It moves like a classical particle but also spreads (dispersion) and can tunnel through barriers (quantum tunneling). The probability density ripples as it moves.

### Key Equations
Time-dependent Schrödinger equation (1D):
```
iℏ ∂ψ/∂t = -(ℏ²/2m)∂²ψ/∂x² + V(x)ψ
```

Crank-Nicolson discretization (implicit, 2nd order, unitary):
```
(1 + iH·dt/2)ψ^{n+1} = (1 - iH·dt/2)ψ^n
```
Where H is the Hamiltonian matrix. This becomes a tridiagonal system solved by the Thomas algorithm.

Probability density: `ρ(x,t) = |ψ(x,t)|² = Re(ψ)² + Im(ψ)²`
Normalization: `Σ_x ρ(x,t)·dx = 1` should be conserved.

### Data Structures
- `psi[N]`: complex float (re[], im[] or complex float[])
- `V[N]`: real potential energy
- Tridiagonal system: a[], b[], c[], d[] for Thomas algorithm
- `rho[N]`: |ψ|² for display

### Non-Obvious Decisions
- **Crank-Nicolson not Euler**: Explicit Euler is unstable for Schrödinger (norm grows). Crank-Nicolson is implicit, unitary — norm is exactly conserved.
- **Thomas algorithm**: The tridiagonal system Ax=b can be solved in O(N) via forward elimination and back-substitution. Store 3 diagonals.
- **Absorbing boundaries**: Multiply ψ by a smooth damping function near the edges: `ψ *= 1 - ε·border_factor`. Prevents reflection off the box walls.
- **Initial state — Gaussian wave packet**: `ψ₀(x) = exp(-((x-x₀)/σ)²/2) · exp(ik₀x)` — a particle with position x₀, width σ, momentum ℏk₀.
- **Visualization**: Plot |ψ|² as a bar chart or filled region. Show Re(ψ) and Im(ψ) oscillating — the quantum phase.

### Key Constants
| Name | Role |
|------|------|
| N | grid points |
| DX | spatial step |
| DT | timestep (Crank-Nicolson is stable for any DT) |
| k₀ | initial wave vector (momentum) |
| σ | initial wave packet width |
| V₀ | barrier height |

### Open Questions
- At what barrier thickness does tunneling probability drop to 1%?
- What happens to the wave packet when V(x) is a harmonic potential?
- Verify: does total norm Σ|ψ|²·dx remain constant throughout simulation?

## From the Source

**Algorithm:** Crank-Nicolson finite-difference scheme. Averages the explicit and implicit Euler steps to give an unconditionally stable, 2nd-order-in-time method. Results in a tridiagonal complex linear system per step, solved with the Thomas algorithm O(N) — not O(N³).

**Physics:** Quantum mechanics, wave-particle duality. ψ(x,t) is complex-valued; |ψ|² is the probability density. The wavefunction spreads (Re, Im oscillate), reflects off walls, and tunnels through barriers with probability `T ≈ exp(−2κd)` where `κ = √(2m(V−E))/ℏ`.

**Math:** Natural units ℏ = m = 1 simplify the Hamiltonian: `H = −½ d²/dx² + V(x)`. DX and DT are in these dimensionless units. The Thomas algorithm (LU decomposition of tridiagonal matrix) gives the next ψ in O(N) operations.

**Performance:** N_GRID=512 points × STEPS_PER_FRAME=20 solves per frame. Each solve is O(N), so total: O(10240) mults per frame — easily runs at 30 fps even on a single core.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_re[N_GRID]`, `g_im[N_GRID]` | `float[512]` × 2 | 4 KB each | real and imaginary parts of the wavefunction ψ |
| `g_V[N_GRID]` | `float[512]` | 2 KB | potential energy array V(x) for the current preset |
| `g_cx_tmp[N_GRID]`, `g_cx_cp[N_GRID]` | `Cx[512]` × 2 | 8 KB each | Thomas algorithm workspace (complex forward-sweep arrays) |
| `g_k0` | `float` | 4 B | initial wavenumber controlling wave packet energy |
| `g_preset` | `int` | 4 B | active preset index (free/barrier/harmonic/double-slit) |
| `g_rows`, `g_cols` | `int` | 8 B | terminal dimensions |

## Pass 2 — Implementation

### Pseudocode
```
init_wavepacket():
    for i in 0..N:
        x = i*DX
        gauss = exp(-((x-x0)/sigma)² / 2)
        psi_re[i] = gauss * cos(k0*x)
        psi_im[i] = gauss * sin(k0*x)
    normalize(psi_re, psi_im)

crank_nicolson_step():
    # Build RHS: b = (I - iH·dt/2) ψ^n
    alpha = -DT/(4*DX²)   # ℏ=m=1
    for i in 1..N-2:
        b_re[i] = psi_re[i] - alpha*(psi_im[i+1]-2psi_im[i]+psi_im[i-1]) + DT/2*V[i]*psi_im[i]
        b_im[i] = psi_im[i] + alpha*(psi_re[i+1]-2psi_re[i]+psi_re[i-1]) - DT/2*V[i]*psi_re[i]

    # Solve tridiagonal (I + iH·dt/2) ψ^{n+1} = b
    thomas_solve(alpha, V, DT, b_re, b_im, psi_re, psi_im)

draw():
    for i in 0..N:
        rho = psi_re[i]²+psi_im[i]²
        bar_height = (int)(rho * scale)
        draw_bar(i, bar_height, rho_color(rho))
    # Overlay Re(psi) as line
    draw_waveform(psi_re, green)
    draw_potential(V, yellow)
```

### Module Map
```
§1 config    — N, DX, DT, k0, sigma, barrier params
§2 init      — init_wavepacket(), normalize()
§3 physics   — crank_nicolson_step(), thomas_solve()
§4 draw      — |ψ|² bars + Re(ψ) wave + V(x) potential
§5 app       — main loop, keys (barrier height, width, reset, momentum)
```

### Data Flow
```
initial Gaussian ψ₀ → Crank-Nicolson → ψ(t)
|ψ|² → probability bars; Re(ψ) → wave overlay; V(x) → potential line
```
