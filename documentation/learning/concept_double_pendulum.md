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

## From the Source

**Algorithm:** 4th-order Runge-Kutta (RK4) integration. Equations of motion derived from the Lagrangian (energy minimisation formulation), reducing to four coupled ODEs for (θ₁, θ₂, ω₁, ω₂). RK4 gives O(dt⁵) local error vs. O(dt²) for symplectic Euler. For a chaotic system this matters: phase errors grow at rate e^(λt) where λ ≈ 1/s is the Lyapunov exponent. Lower-order integrators lose tracking within 2–3 s.

**Math:** D = 3 − cos 2(θ₁−θ₂) ≥ 2 (denominator, never zero). The derived angular accelerations α₁, α₂ are computed in `state_deriv()`. Sim runs at 300 Hz; display at ~60 Hz. Between sim ticks angles are linearly interpolated by alpha ∈ [0,1], giving smooth motion without needing 300-Hz rendering.

**Performance:** RK4 global error O(dt⁴) ≈ (3.3e-3)⁴ ≈ 1e-10 per step at 300 Hz sim rate — negligible for few-second time-scales. At 300 Hz, dt ≈ 3.3 ms per RK4 step.

**Physics/References:** The ghost pendulum (GHOST_EPSILON offset on θ₁) starts indistinguishably close to the primary. After ~3–5 s (one Lyapunov time) the two trajectories diverge visibly — a direct demonstration that tiny measurement errors become arbitrarily large. Ring-buffer trail (TRAIL_LEN positions) iterated from oldest to newest by offset arithmetic — no shifting.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~10 KB | top-level container: scene (DPend × 2 + Trail) + screen + control flags |
| `g_app.scene.primary` / `.ghost` | `DPend` | ~40 B each | RK4 state (t1, t2, ω1, ω2), prev-tick angles, pivot, arm length |
| `g_app.scene.trail` | `Trail` | ~4 KB | ring-buffer of TRAIL_LEN=500 end-bob pixel positions |
