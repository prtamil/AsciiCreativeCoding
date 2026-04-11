# Concept: Gyroscope (Rigid Body Rotation)

## Pass 1 — Understanding

### Core Idea
A spinning body resists changes to its rotation axis (gyroscopic effect). When gravity pulls on a tilted spinning top, instead of falling it precesses — the axis slowly rotates around the vertical. The faster the spin, the slower the precession.

### Mental Model
A bicycle wheel spinning fast stays upright when you tilt it. If you hold one axle end and let it go, it circles slowly instead of falling. That circular drift is precession — the angular momentum vector chases gravity's torque.

### Key Equations
```
Torque:    τ = r × F  (cross product of arm × gravity)
Angular momentum: L = I · ω  (I = moment of inertia)
Precession rate:  Ω = τ / L = Mgr / (I·ω)
Nutation: wobble on top of precession (higher-order effect)
```

Euler angles:
- φ (phi): spin angle of the disc
- θ (theta): tilt from vertical
- ψ (psi): precession angle around vertical

### Data Structures
- State: (phi, theta, psi, dphi, dtheta, dpsi)
- Euler's equations of motion for rigid body
- RK4 integrator over 6D state vector

### Non-Obvious Decisions
- **Euler angles have gimbal lock at θ=0**: The equations become singular when the top is perfectly upright. Add a small initial tilt.
- **Two moments of inertia**: Symmetric top has I₁=I₂ (lateral) and I₃ (spin axis). These must be set consistently.
- **Render in 3D**: Draw the disc ellipse and spin axis in the 2D terminal with a 3D→2D projection.
- **Energy check**: Total energy (rotational KE + PE) should be conserved. Drift reveals integrator error.

### Key Constants
| Name | Role |
|------|------|
| I1, I3 | moments of inertia (lateral vs. spin axis) |
| M | mass |
| r | distance from pivot to center of mass |
| ω₀ | initial spin rate (rad/s) |
| θ₀ | initial tilt angle |

### Open Questions
- As spin rate ω₀ → 0, what happens? (top falls)
- Nutation frequency: does it match the theoretical value?
- Can you add air drag and show the top slowly precessing to a halt?

---

## Pass 2 — Implementation

### Pseudocode
```
# Euler's equations for symmetric top (Lagrangian form)
# State: [phi, theta, psi, phi_dot, theta_dot, psi_dot]

derivs(state):
    phi=state[0], th=state[1], psi=state[2]
    dp=state[3], dt=state[4], ds=state[5]

    # Euler-Lagrange equations for symmetric top
    I1*th_ddot = (I1*dp²*sin(th)*cos(th)
                  - I3*(dp*cos(th)+ds)*dp*sin(th)
                  + M*g*r*sin(th))
    dp_dot = d/dt[phi_dot]  (from conserved Lz)
    ds_dot = d/dt[psi_dot]  (from conserved Lspin)
    return [dp, dt, ds, dp_dot, th_ddot, ds_dot]

rk4_step(state, dt):
    standard 4-stage RK4 over derivs()

project_disc(phi, theta, psi) → ellipse params:
    # rotate spin axis by Euler angles
    # project to screen, draw ellipse with '*' and '─'
```

### Module Map
```
§1 config    — I1, I3, M, r, g, initial angles/rates
§2 physics   — derivs(), rk4_step()
§3 render    — Euler→rotation matrix → 3D disc → 2D ellipse
§4 draw      — ellipse chars, axis line, trail of tip
§5 app       — main loop, keys (spin rate, tilt, pause)
```

### Data Flow
```
initial Euler angles + rates → rk4 → new state → rotation matrix
→ disc vertices in 3D → project to 2D → draw ellipse + axis
```
