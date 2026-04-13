# Next 20 Programs

Ideas ranked by learning value and visual payoff. Each builds on existing project knowledge.

---

## Tier 1 — Visual Math / Particle Art

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 1 | ~~`galaxy.c`~~ | artistic/ | Spiral galaxy — stars orbit a central mass with Newtonian gravity + bar arm pattern | Polar spiral parametric, N-body simplified, particle trail with lifetime fade | **DONE** |
| 2 | `sand_art.c` | artistic/ | Sand falling through a rotating hourglass — gravity CA + rotating obstacle | Falling sand CA + angle-driven gravity vector, collision geometry |
| 3 | ~~`fourier_art.c`~~ | artistic/ | Real-time Fourier drawing — user traces a path, DFT computes epicycles that redraw it | DFT on sampled path, epicycle arm chain sorted by amplitude | **DONE** |
| 4 | ~~`magnetic_field.c`~~ | physics/ | 2D magnetic field lines from configurable dipoles — streamline tracing | Vector field integration, Biot-Savart for discrete charges | **DONE** |
| 5 | `wave_interference.c` | fluid/ | N point sources radiating in a tank — real-time amplitude sum with color map | FDTD or analytic sum, signed amplitude → color gradient |

---

## Tier 2 — Physics Simulations

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 6 | `bubble_chamber.c` | physics/ | Charged particles curving in a magnetic field — particle tracks with decay | Lorentz force (v × B), exponential trail fade, track curvature |
| 7 | ~~`chain.c`~~ | physics/ | Hanging chain + swinging rope — verlet constraint solver | Position-based dynamics, distance constraints, iterative projection | **DONE** |
| 8 | `fluid_sph.c` | fluid/ | Smoothed Particle Hydrodynamics — 2D particle fluid with pressure and viscosity | SPH density, pressure gradient, viscosity term, neighbour search |
| 9 | ~~`rigid_body.c`~~ | physics/ | 2D rigid body collisions — cubes + spheres, AABB detection, gravity, sleep | AABB overlap, circle-AABB closest-point, impulse resolution, Coulomb friction, Baumgarte, sleep counter | **DONE** |
| 10 | ~~`soft_body.c`~~ | physics/ | Jelly blob — spring lattice with surface tension | Spring-mass mesh, surface rendering, pressure term for volume conservation | **DONE** |

---

## Tier 3 — Fractals & Chaos

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 11 | `julia_explorer.c` | fractal_random/ | Interactive Julia set explorer — arrow keys move the c parameter in real time | Same julia escape-time code, but c updates live and grid recomputes incrementally |
| 12 | `barnsley.c` | fractal_random/ | Full Barnsley IFS system — multiple transforms with arbitrary affine matrices | IFS transform table, probability-weighted random selection, chaos game |
| 13 | `diffusion_map.c` | fractal_random/ | Eden growth model — cells grow from a seed by random attachment (no symmetry) | Simple DLA without symmetry, natural blob morphology |
| 14 | `tree_la.c` | fractal_random/ | Lightning-tree fractal — dielectric breakdown model (DBM) for lightning branches | DBM probability field (Laplace-like), probabilistic attachment, recursive branching |
| 15 | `lyapunov.c` | fractal_random/ | Lyapunov fractal — parameter space of a logistic map with alternating parameters | Alternating a/b sequences, Lyapunov exponent per pixel, signed color map |

---

## Tier 4 — Emergent & Cellular Systems

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 16 | ~~`slime_mold.c`~~ | fluid/ | Physarum polycephalum — agent-based slime that creates optimal networks | Agent sensors (3-direction sample), deposit trail, diffuse + decay trail grid | **DONE** |
| 17 | `vote_ca.c` | misc/ | Voter model / Schelling segregation — cells adopt neighbour majority state | Majority rule CA, threshold-based update, spatial opinion dynamics |
| 18 | ~~`forest_fire.c`~~ | misc/ | Forest fire cellular automaton — tree growth + lightning ignition + fire spread | 3-state CA (empty/tree/fire), probabilistic growth and ignition rates | **DONE** |
| 19 | ~~`lattice_gas.c`~~ | fluid/ | FHP lattice gas automaton — particle collisions on hexagonal lattice | 6-direction bit-coded state, collision lookup table, Frisch-Hasslacher-Pomeau rules | **DONE** |
| 20 | `excitable.c` | fluid/ | Greenberg-Hastings excitable medium — rotating spiral waves, 3 states | 3-state CA (resting/excited/refractory), spiral wave nucleation |

---

## Notes

- **galaxy.c** and **fourier_art.c** are the highest visual payoff for least new math — build these first.
- **fluid_sph.c** is the hardest — requires neighbour search (spatial hashing) and tuned pressure constants; attempt after navier_stokes.c is understood.
- **slime_mold.c** is agent-based (like flocking.c) but with a trail grid (like flowfield.c) — combine both patterns.
- **julia_explorer.c** is a 1-hour add-on to the existing julia.c — incremental recompute only for changed c.
- **tree_la.c** extends lightning.c with a Laplace-like probability field instead of random branching — the DBM model produces more realistic tree-lightning shapes.
