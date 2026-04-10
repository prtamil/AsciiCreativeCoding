# Pass 1 — physics/double_pendulum.c: Chaotic Double Pendulum

## Core Idea

A pendulum hanging from another pendulum. Two rigid rods, two equal masses, no friction. The equations of motion are derived from the Lagrangian (energy conservation) and integrated with 4th-order Runge-Kutta. A "ghost" pendulum starts with an imperceptibly different angle and tracks the same equations — at first it overlaps the main pendulum exactly, but after a few seconds the two trajectories diverge completely, demonstrating chaos.

## The Mental Model

Imagine holding a ruler by one end, and at the other end of the ruler a second ruler hangs freely. If you give it a gentle push, the second ruler swings back and forth predictably. But if you push harder so the second rod goes over the top, even a tiny difference in how hard you pushed leads to completely different motion within a few seconds. This is chaos: bounded, deterministic, yet unpredictable beyond a short horizon.

## Equations of Motion (Lagrangian Derivation)

With equal masses m₁ = m₂ = 1, equal arm lengths L₁ = L₂ = L, let δ = θ₁ − θ₂ and D = 3 − cos 2δ (always ≥ 2):

    θ₁'' = [−3g sin θ₁ − g sin(θ₁−2θ₂) − 2 sin δ (ω₂²L + ω₁²L cos δ)] / (L·D)
    θ₂'' = [2 sin δ (2ω₁²L + 2g cos θ₁ + ω₂²L cos δ)] / (L·D)

`D ≥ 2` always, so there are no singularities.

## RK4 Integration

The state vector is `[θ₁, θ₂, ω₁, ω₂]`. One RK4 step:

    k1 = f(state)
    k2 = f(state + dt/2 · k1)
    k3 = f(state + dt/2 · k2)
    k4 = f(state + dt · k3)
    state += dt/6 · (k1 + 2·k2 + 2·k3 + k4)

4th-order means the local error is O(dt⁵). For chaotic systems, lower-order methods (Euler, Verlet) produce phase errors on the Lyapunov time-scale (~3–5 s) that are visually indistinguishable from genuine chaos — using RK4 ensures the simulated chaos is real, not numerical artefact.

## Ghost Pendulum

A second pendulum starts with `θ₁ + GHOST_EPSILON` (a tiny angle offset, e.g. 0.001 rad). Both are integrated with the same equations, independently. At first they are visually identical. After a few Lyapunov times they diverge completely — the angular separation grows exponentially. The HUD shows this separation, making the Lyapunov exponent tangible.

## Ring-Buffer Trail

The second bob's position history is stored in a `TRAIL_LEN`-entry ring buffer. Each draw cycle, the trail is rendered from oldest to newest:
- Oldest entries: dim grey
- Middle entries: orange
- Newest entries: bright red

The trail reveals the attractor geometry — the complex tangle of paths that the chaotic system traces over time.

## Non-Obvious Decisions

### Why equal masses and arm lengths?
The simplified equal-mass, equal-length case eliminates two free parameters and produces the most symmetric equations while still being fully chaotic. The visual attractor looks symmetric about the vertical axis for many initial conditions.

### Why GHOST_EPSILON = 0.001 radians?
Small enough that the two pendulums are visually indistinguishable for the first few seconds, large enough that divergence occurs within a reasonable viewing time (~5–10 s). Too small: divergence takes too long; too large: divergence is immediate and the demo loses impact.

### Why `D = 3 − cos 2δ` never has singularities?
For a double pendulum the effective mass matrix determinant determines D. Since both masses are equal and positive, D is always ≥ 2 — the denominator never reaches zero regardless of the configuration.

## Key Constants

| Constant | Effect |
|---|---|
| L (arm length) | Longer arms → slower oscillation, larger attractor |
| g (gravity) | Higher → faster oscillation, more chaotic |
| GHOST_EPSILON | Smaller → divergence takes longer; larger → immediate divergence |
| TRAIL_LEN | Longer trail reveals more attractor geometry |
| dt (time step) | Smaller → more accurate but slower sim steps per frame |
