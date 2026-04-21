# Concept: ODE Integrator Comparison (Euler / RK2 / RK4 / Verlet)

## Pass 1 — Understanding

### Core Idea
Compare four numerical methods for integrating ODEs on Hamiltonian test systems (harmonic oscillator, nonlinear pendulum). The key insight: accuracy order (how fast error shrinks with smaller step size) and geometric properties (energy conservation, symplecticity) are independent. RK4 is the most accurate per-step but drifts for very long integrations. Velocity Verlet is only second-order but never drifts because it is symplectic.

### Mental Model
Imagine steering a car along a curved road in the dark. Euler: look at where you're pointing right now and drive straight for 1 second. You overshoot every curve. RK2: peek halfway ahead first, then use that direction. Better. RK4: take four steering readings, weight them carefully. Very accurate. Verlet: always update speed before position (not simultaneously). This seemingly small change preserves the "balance" in the equations — energy doesn't creep in or out over time.

### Key Equations
```
Hamiltonian H = ½p² + V(q)
Hamilton's equations: dq/dt = p,  dp/dt = −∂V/∂q

Euler:       p_{n+1} = p_n + h·F(q_n)
             q_{n+1} = q_n + h·p_n

RK4 (4-stage):
  k1 = f(q_n,         p_n)
  k2 = f(q_n + h/2·k1.dq, p_n + h/2·k1.dp)
  k3 = f(q_n + h/2·k2.dq, p_n + h/2·k2.dp)
  k4 = f(q_n + h·k3.dq,   p_n + h·k3.dp)
  (q,p)_{n+1} = (q,p)_n + h/6·(k1 + 2k2 + 2k3 + k4)

Velocity Verlet:
  p_half = p_n + h/2·F(q_n)
  q_{n+1} = q_n + h·p_half
  p_{n+1} = p_half + h/2·F(q_{n+1})
```

### Data Structures
- 4 State structs `{q, p}`, one per method
- 4 ring-buffer trajectory arrays for phase portrait
- 4 energy time series arrays for drift plot
- Reference state: RK4 with 32 substeps per display step

### Non-Obvious Decisions
- **Verlet not RK2 for long-time**: Both are O(h²) but Verlet is symplectic (preserves phase-space volume). Over thousands of orbits RK2 drifts; Verlet oscillates bounded.
- **Energy error not position error**: For Hamiltonian systems energy conservation is the natural metric. Position error after many orbits is meaningless (orbit phase shifts); energy tells you if the method is physically consistent.
- **Reference solution**: RK4 at 32× substeps gives error O((h/32)⁴) ≈ 5×10⁻⁸× the display-step error — effectively exact for visual comparison.
- **Side-by-side not overlaid**: Four trajectories in one phase portrait would be unreadable. Four panels let you see each method's behavior independently (Euler spiral vs Verlet closed curve).

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| DT_DEFAULT | 0.05 | display step (ωh = 0.05 rad for ω=1) |
| OMEGA | 1.0 | oscillator natural frequency |
| Q0_PEND | 1.40 | pendulum initial angle (80° — nonlinear regime) |
| REF_SUBSTEPS | 32 | RK4 substeps for reference solution |
| TRAJ_CAP | 512 | ring buffer capacity for phase portrait |

### Open Questions
- At what step size does Euler become visually distinguishable from Verlet?
- Why does RK4 also eventually drift (just much slower)?
- What is the stability region of each method in the complex plane?

## From the Source

**Algorithm:** Four integrators running simultaneously on shared ODE. Hamiltonian systems: harmonic oscillator and nonlinear pendulum. Energy metric per step: H(t) − H(0). Reference via RK4×32 substeps.

**Physics/References:** Symplectic integrators: Leimkuhler & Reich "Simulating Hamiltonian Dynamics" (2004). Velocity Verlet equivalence to Störmer-Verlet: Verlet (1967). Stability analysis: Hairer, Lubich & Wanner "Geometric Numerical Integration" (2006).

**Math:** Amplification factor for Euler on harmonic oscillator: |A|² = 1+(ωh)². For ωh=0.05: |A|=1.00125/step. After 1000 steps: |A|^1000 = 3.49 — 3.5× energy growth visible in phase portrait.

**Performance:** 4 integrators × 1 ODE eval per step (plus RK4 = 4 evals × 32 substeps = 128 evals for reference). Total: ~140 float operations per display tick.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `state[4]` | `State {q,p}[4]` | 4 × 2 floats | current (q,p) per method |
| `traj[4][TRAJ_CAP]` | `State[4×512]` | ~16 KB | ring-buffer trajectories |
| `energy[4][TSER_CAP]` | `float[4×1024]` | ~16 KB | energy drift time series |
| `ref_state` | `State` | 2 floats | high-accuracy reference |

---

## Pass 2 — Implementation

### Pseudocode
```
tick():
    for each method m:
        state[m] = integrate(state[m], DT, method=m)
        H = hamiltonian(state[m])
        push_energy(energy[m], (H - H0) / H0 * 100)
        push_traj(traj[m], state[m])

    # reference: RK4 with 32 substeps
    h_sub = DT / REF_SUBSTEPS
    for _ in REF_SUBSTEPS: ref_state = rk4(ref_state, h_sub)
    push_traj(traj[REF], ref_state)

euler(s, h):
    return {q: s.q + h*s.p, p: s.p + h*(-dV_dq(s.q))}

rk4(s, h):
    k1 = f(s)
    k2 = f(s + h/2*k1)
    k3 = f(s + h/2*k2)
    k4 = f(s + h*k3)
    return s + h/6*(k1 + 2*k2 + 2*k3 + k4)

verlet(s, h):
    p_half = s.p + h/2*(-dV_dq(s.q))
    q_new  = s.q + h*p_half
    p_new  = p_half + h/2*(-dV_dq(q_new))
    return {q: q_new, p: p_new}
```

### Module Map
```
§1 config       — DT, OMEGA, Q0, system choice (oscillator/pendulum)
§2 ODE systems  — hamiltonian(), dV_dq() for both systems
§3 integrators  — euler(), rk2(), rk4(), verlet()
§4 ring buffers — traj_push/read, energy_push/read
§5 render       — 4 phase portrait panels + energy drift panel
§6 HUD          — energy values, method names, step size
§7 app          — main loop, system/dt controls
```

### Data Flow
```
state[4] → integrate → state[4] (new)
state[4] → hamiltonian → energy[4] (drift)
state[4] → traj_push → traj[4] (phase portrait)
traj[4] → render_panels → screen
energy[4] → render_timeseries → screen
```
