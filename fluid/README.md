# fluid — solving partial differential equations on a terminal grid

A reference folder for **continuous-medium simulation**: 11 self-contained
C programs that take a partial differential equation (Navier-Stokes,
shallow-water, Gray-Scott, FitzHugh-Nagumo, the wave equation) and turn
it into colour glyphs at 60 fps. Every file lives in the same intellectual
neighbourhood — a **field on a grid**, updated each tick by a **stencil**
of nearby values — and the folder's whole job is to lay out that
neighbourhood from four complementary angles:

1. **Eulerian PDE solvers** — `navier_stokes.c`, `nuke.c`,
   `shallow_water_solver.c`, `vorticity_streamfunction_solver.c`. Discretise
   a continuum equation; iterate.
2. **Reaction systems** — `reaction_diffusion.c` (Gray-Scott),
   `reaction_wave.c` (FitzHugh-Nagumo). Per-cell ODE coupled to a
   diffusion stencil. No advection.
3. **Particle / lattice alternatives** — `fluid_sph.c` (Lagrangian SPH),
   `lattice_gas.c` (FHP-I hexagonal cellular automaton). Different
   philosophy, same macroscopic flow.
4. **Field-as-lookup** — `flowfield.c`, `complex_flowfield.c`. No PDE at
   all; an analytic / noise-driven vector field drives tracer particles.
5. **The meta-tool** — `cfl_stability_explorer.c`. Not a demo: a live
   visualiser of *why* every file above has to think about the time-step
   limit, built around the simplest possible 2-D wave equation.

If you read **only one file**, read [`navier_stokes.c`](navier_stokes.c) —
Jos Stam's 1999 "Stable Fluids" is the canonical algorithm of the whole
folder, and almost every other Eulerian file here borrows a piece of it.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
3. [File index](#file-index)
4. [Building and running](#building-and-running)
5. [Adding a new solver](#adding-a-new-solver)
6. [Notes on duplicated demos](#notes-on-duplicated-demos)

---

## How to read this folder

The recommended path goes from *no PDE at all* to *full Navier-Stokes*,
then branches into specialised solvers and the stability meta-tool.

```
                    EULERIAN ladder
                    ───────────────
   1.  flowfield.c                       — no PDE, noise → vector field → advect tracers
            │
            ▼
   2.  complex_flowfield.c               — same tracer engine, FOUR field generators
            │
            ▼
   3.  reaction_diffusion.c              — first PDE: two scalar fields, diffusion + reaction
            │
            ▼
   4.  reaction_wave.c                   — excitable medium (FitzHugh-Nagumo)
            │
            ▼
   5.  shallow_water_solver.c            — coupled (h, u, v) fields, linearised SWE
            │
            ▼
   6.  navier_stokes.c                   — full Stam stable fluids (THE keystone)
            │              │
            ▼              ▼
   7. vorticity_streamfunction_solver.c  7b. nuke.c
       same physics, ω-ψ formulation         buoyant fluid + volumetric raymarch

                    PARTICLE / LATTICE alternatives
                    ────────────────────────────────
   A.  fluid_sph.c                       — Lagrangian Smoothed Particle Hydrodynamics
   B.  lattice_gas.c                     — discrete FHP-I cellular automaton

                    META-TOOL
                    ─────────
   M.  cfl_stability_explorer.c          — visualise the time-step bound that limits A-7
```

**Prerequisites graph.** Each file's header lists *Study alongside:*
pointers. The two most important cross-references:

* The four Eulerian solvers (3, 5, 6, 7) all share the **operator-split**
  pattern (`diffuse → advect → project` in some order). Read the bodies
  of any two and the third becomes obvious.
* SPH (A) and lattice gas (B) are **deliberate counterpoints** to the
  grid solvers. T1 in both files explains the Eulerian/Lagrangian split.

**Background you need.** Some linear algebra (gradients, Laplacians,
divergence), comfort with `float` arithmetic, and the framework
conventions (fixed-step accumulator, `erase()/doupdate()` cycle) from
[`../grids/README.md`](../grids/README.md). No prior fluid-dynamics
background is assumed; every file's MENTAL MODEL block teaches the
physics from scratch.

---

## The unifying primitive

Every file in this folder is built around the same two-step pattern:

```
            STATE                                  UPDATE
        ─────────────                          ──────────────
   scalar or vector FIELD     ─────tick──▶     STENCIL of neighbours
   on a 2-D grid                                + per-cell reaction terms
   (or particle pool                            (or particle interaction)
    + spatial hash)                                       │
                                                          ▼
                                                  new field / new positions
```

Concretely the four families realise it like this:

| Family            | State                                          | Per-tick update                                   |
|-------------------|------------------------------------------------|---------------------------------------------------|
| Stam / Stable     | velocity `(u, v)` + scalar `dye / T / ρ`       | `diffuse → advect → project` (operator split)     |
| Reaction          | two scalars `(U, V)` or `(u, v)`               | Laplacian stencil + non-linear local reaction     |
| Wave / SWE        | three fields (`h`, `u`, `v`) or `(past, now)`  | central differences, explicit time-stepping       |
| SPH / lattice gas | particle pool / per-cell bit pattern           | neighbour sum (kernel) / table lookup + stream    |
| Field-as-lookup   | noise / Biot-Savart angle grid                 | analytic re-evaluation each frame                 |

The **Stam split** — the move that made `navier_stokes.c`, `nuke.c`, and
`vorticity_streamfunction_solver.c` possible — looks like this in ASCII:

```
   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
   │ DIFFUSE  │───▶│ PROJECT  │───▶│ ADVECT   │───▶│ PROJECT  │
   │  (μ∇²v)  │    │ (∇·v=0)  │    │ (back-   │    │ (∇·v=0)  │
   │ implicit │    │ Poisson  │    │  trace)  │    │ Poisson  │
   └──────────┘    └──────────┘    └──────────┘    └──────────┘
        ▲                                                │
        └────────────────────────────────────────────────┘
                            ONE TICK
```

Each phase is **unconditionally stable**: crank `dt` and the fluid
blurs, never explodes. This is why the Stam files don't appear in
`cfl_stability_explorer.c`'s gallery — they sidestep the CFL bound by
construction. The explicit files (shallow water, reaction-diffusion,
reaction-wave) **do** live under a CFL bound, and the meta-tool exists
to make that bound visible.

**Reaction terms.** For files 3 and 4 the operator split disappears and
the update collapses to a single line per cell:

```
   reaction_diffusion (Gray-Scott):
       dU/dt = Du · ∇²U  −  U·V²  +  f·(1 − U)
       dV/dt = Dv · ∇²V  +  U·V²  −  (f + k)·V

   reaction_wave (FitzHugh-Nagumo, excitable medium):
       du/dt = u − u³/3 − v + D · ∇²u + I(t)
       dv/dt = ε · (u + a − b · v)
```

Two scalars per cell, a 5- or 9-point Laplacian stencil, forward Euler.
The pattern formation comes from the **non-linear reaction term**, not
from any fluid motion.

---

## File index

| File                                       | Lines | Family       | DEMO line — what it visually does                                                                   |
|--------------------------------------------|-------|--------------|------------------------------------------------------------------------------------------------------|
| `flowfield.c`                              | ~1700 | field-lookup | Tracer particles drift on a Perlin-noise vector field; toggle arrows for the wind beneath.           |
| `complex_flowfield.c`                      | ~2000 | field-lookup | Same tracer engine, four pluggable field generators: curl noise / vortex lattice / sine / spiral.    |
| `reaction_diffusion.c`                     | ~1800 | reaction     | Gray-Scott `(f, k)` Turing patterns: spots, stripes, coral, mazes, solitons. Seven presets.          |
| `reaction_wave.c`                          | ~1700 | reaction     | FitzHugh-Nagumo excitable medium: target rings, spiral waves, plane waves; threshold + refractory.   |
| `shallow_water_solver.c`                   | ~2100 | explicit PDE | Linearised 2-D shallow water `(h, u, v)`: dam break, drop, channel, obstacle scattering.             |
| `navier_stokes.c`                          | ~1900 | Stam split   | Stam 1999 stable fluids: two counter-rotating dye emitters, swirling plumes, arrow-key forcing.      |
| `nuke.c`                                   | ~2200 | Stam split   | Mushroom cloud: 2-D axisymmetric buoyant Stam fluid + volumetric Beer-Lambert raymarcher.            |
| `vorticity_streamfunction_solver.c`        | ~2300 | ω-ψ          | 2-D NS in vorticity-streamfunction form: Karman street, lid cavity, free jet, backward step.        |
| `fluid_sph.c`                              | ~1700 | Lagrangian   | 800-3000 SPH particles: blob, fountain, slamming columns, rain. Density estimation + spatial hash.   |
| `lattice_gas.c`                            | ~1800 | discrete CA  | FHP-I hex lattice gas: six bits per cell + collision table → emergent Navier-Stokes flow.            |
| `cfl_stability_explorer.c`                 | ~1700 | meta-tool    | Watch the leapfrog wave equation explode the instant `c·dt/dx` crosses 1. Six CFL presets.           |

Line counts include the pedagogical block (HOW TO READ, GUIDED TUTORIAL,
inline teaching prose) — the algorithm itself is typically 400-700 lines.

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra fluid/<file>.c -o <name> -lncurses -lm
```

All eleven files use `-lm` (every PDE solver needs `sqrt`, `cos`, `exp`,
`expf`, etc.). The project is strict about `-Wall -Wextra` clean — every
file compiles with zero warnings.

**Universal keys** (always present):

| Key             | Action                                |
|-----------------|---------------------------------------|
| `q` / `ESC`     | quit                                  |
| `space` / `p`   | pause                                 |
| `r`             | reset / reload current scene          |
| `t` / `T`       | next / previous theme                 |
| `+` / `-`       | tune the headline parameter           |

**Solver-specific keys** (vary per file):

| Key             | Where                                  | Action                                          |
|-----------------|----------------------------------------|-------------------------------------------------|
| `1`..`9`        | most solvers                           | jump between presets / scenes                   |
| arrows          | `navier_stokes.c`                      | push velocity at the centre                     |
| `d`             | `navier_stokes.c`, `nuke.c`, others    | drop dye / cycle debug overlay                  |
| `D`             | most files                             | numeric overlay (raw field values)              |
| `a`             | `flowfield.c`, `complex_flowfield.c`   | toggle field-arrow background / cycle field     |
| `g`, `v`        | `fluid_sph.c`, `shallow_water_solver.c`| gravity / viscosity toggle                      |
| `x`             | `vorticity_streamfunction_solver.c`    | toggle tracer-particle overlay                  |
| `m`             | `cfl_stability_explorer.c`             | eigenmode initial condition (clean instability) |

---

## Adding a new solver

If you want to add (say) a heat-equation solver, a Burgers-equation
shock tube, or a MHD demo:

1. **Pick the philosophy.** Eulerian grid (most files), Lagrangian
   particles (SPH), or discrete CA (lattice gas)?  Different state
   layouts, different update patterns.
2. **Decide on operator splitting vs monolithic step.** If your PDE has
   stiff terms (high diffusion, pressure projection), copy the Stam
   split from `navier_stokes.c`. If it's purely hyperbolic + mild
   diffusion, copy the central-difference loop from
   `shallow_water_solver.c`.
3. **Compute the CFL bound by hand** before writing the time loop. Use
   `cfl_stability_explorer.c` to confirm visually that crossing the
   bound blows up the simulation.
4. **Pick a 5-point or 9-point Laplacian.** 9-point is rounder (no
   diamond artefacts in radial patterns); 5-point is faster.
   `reaction_diffusion.c` uses 9-point; `reaction_wave.c` uses 5-point;
   `navier_stokes.c` uses 5-point inside Gauss-Seidel.
5. **Copy the `01`-style template:**
   * Stam-split fluid → copy `navier_stokes.c`
   * Reaction system → copy `reaction_diffusion.c`
   * Explicit hyperbolic → copy `shallow_water_solver.c`
   * Particle method → copy `fluid_sph.c`
6. **Add CONCEPTS + MENTAL MODEL** per the project's
   [CLAUDE.md](../CLAUDE.md) template. Both blocks are mandatory.
   References: 2-5 per file.
7. **Verify:** `-Wall -Wextra` clean, stable 60 fps, `q`/`ESC` exits
   cleanly, `SIGWINCH` doesn't crash, HUD shows fps + parameters + state.

---

## Notes on duplicated demos

### `reaction_diffusion.c` lives **here** AND in `procedural/fields/`

`fluid/reaction_diffusion.c` and
`procedural/fields/reaction_diffusion_gray_scott.c` implement the **same**
Gray-Scott model — and they coexist intentionally.

| Angle                          | File                                                      |
|--------------------------------|-----------------------------------------------------------|
| **PDE / numerics**             | `fluid/reaction_diffusion.c` (this folder)                |
|   Frames the model as a discrete-Laplacian explicit-Euler solver. |                                |
|   Emphasises the 9-point isotropic stencil, the CFL bound, the    |                                |
|   activator-inhibitor PDE form. Tutorials walk through the math. |                                |
| **Patterns-as-field**          | `procedural/fields/reaction_diffusion_gray_scott.c`       |
|   Frames the model as one of several pattern-generating fields    |                                |
|   (alongside Perlin noise, curl noise, Voronoi). 5-point stencil, |                                |
|   five presets, lighter pedagogy. Lives next to its visual peers. |                                |

If you want to learn the *equation*, read this folder. If you want to
learn how to *use* the patterns in larger visual compositions, read the
procedural version. Both compile to the same on-screen shapes.

### `reaction_wave.c` is **FitzHugh-Nagumo**, not Gray-Scott

A small but important distinction: `reaction_wave.c` is an **excitable
medium** model (FitzHugh-Nagumo, 1961) — the math of nerve impulses,
heart muscle, and Belousov-Zhabotinsky spirals. It looks superficially
similar to `reaction_diffusion.c` (two scalar fields, diffusion +
reaction), but the physics is fundamentally different:

| Property             | Gray-Scott (`reaction_diffusion.c`) | FitzHugh-Nagumo (`reaction_wave.c`) |
|----------------------|-------------------------------------|-------------------------------------|
| Steady state         | Stationary patterns (spots, stripes)| Quiescent + travelling waves        |
| Both fields diffuse? | Yes (`Du` ≠ `Dv`, both > 0)         | Only `u` diffuses; `v` is local     |
| Threshold behaviour  | No                                  | Yes — sub-threshold inputs decay    |
| Refractory period    | No                                  | Yes — explicit                      |
| Bistability source   | Autocatalytic `U·V²` term           | Cubic `u − u³/3`                    |

Both files share a numerical pattern (5-point Laplacian + forward Euler)
but they teach different stories about non-linear PDEs.

### `cfl_stability_explorer.c` is a **tool**, not a demo

The other ten files in this folder *use* the CFL condition implicitly —
they pick a `dt` that satisfies it and run. `cfl_stability_explorer.c`
inverts the relationship: it makes the bound itself the subject of the
visualisation. Six presets walk the CFL number from 0.4 (deeply stable,
ripples spread cleanly) to 1.4 (unstable, amplitudes blow up to NaN).
It's the file you read when you need to convince yourself, on a particular
machine and font and terminal, that the bound is real and where it bites.

If you have just written a new explicit solver and the screen filled
with garbage, this is the file to read second (after `navier_stokes.c`
to see how *implicit* schemes sidestep the bound entirely).

---

## Cross-references

* [`../grids/README.md`](../grids/README.md) — `GridCtx` and the
  cell ↔ screen mapping every solver uses.
* [`../procedural/README.md`](../procedural/README.md) — the patterns-as-
  field angle on Gray-Scott; also home to Perlin/curl-noise field
  generators that `flowfield.c` / `complex_flowfield.c` borrow from.
* [`../animation/`](../animation/) — the framework conventions
  (`fixed-step accumulator`, `erase()/doupdate()`) used by every
  solver here.
* [`../raster/`](../raster/) — `nuke.c` shares its volumetric raymarcher
  with the raster pipeline; read both for the
  surface-raymarch / volume-raymarch contrast.
* [`../documentation/Architecture.md`](../documentation/Architecture.md)
  — framework deep dive; relevant when grafting a new solver onto the
  standard loop.
