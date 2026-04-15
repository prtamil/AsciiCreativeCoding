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

## From the Source

**Algorithm:** RK4 integration of a 7-component state vector: [ωx, ωy, ωz] — angular velocity in body frame (rad/s); [qw, qx, qy, qz] — orientation quaternion (unit length). RK4 is used because Euler equations are nonlinear: the cross-products (I₂−I₃)ωy·ωz etc. make the ODE stiff.

**Physics:** Euler's equations of rigid-body rotation. In the body frame (principal axes), the inertia tensor is diagonal with eigenvalues I₁, I₂, I₃. The intermediate-axis theorem: rotation near the SMALLEST or LARGEST inertia axis is stable; rotation near the MIDDLE inertia axis is unstable (Dzhanibekov effect).

**Math:** Quaternion orientation tracking. A quaternion q = (qw, qx, qy, qz) with |q|=1 encodes 3-D rotation without gimbal lock or singularities. The rotation matrix R(q) is derived analytically. After each RK4 step q is re-normalised: `q /= |q|`. Gram-Schmidt re-orthogonalises the extracted axes to prevent floating-point accumulation errors.

**Performance:** Orthographic projection of 3D wireframe (ring + axes). ASPECT = CELL_W/CELL_H ≈ 0.5 compensates for non-square terminal cells so circles appear round not elliptical. SUB_STEPS=8 RK4 sub-steps per sim tick for stability.

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
