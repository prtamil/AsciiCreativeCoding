# Learning Roadmap — ASCII Creative Coding

89 C files, 40+ topics. This roadmap gives the optimal study order, the per-file loop,
and the 2-year breakdown. Do not skip layers.

---

## The Learning Loop (do this for every single file)

```
1. MATH FIRST  — find the original paper/algorithm, understand the equation on paper
2. READ CODE   — map every code block to the equation it implements
3. ANNOTATE    — rewrite comments in your own words explaining WHY not WHAT
4. BREAK IT    — change one constant at a time, predict before running
5. REWRITE     — close the file, rewrite from scratch using only the math source
6. VARY        — add one thing the original did not do (this is where it becomes yours)
```

**The single most important rule:** Never run the program before you can predict its output.

---

## Tier 1 — Visual Math (2–3 months)
Needs only geometry and trigonometry. Fast feedback loops.

| File | Subject | Key Math |
|---|---|---|
| `cellular_automata_1d.c` | Wolfram CA | bitwise rule lookup |
| `life.c` | Conway GoL | Moore neighborhood |
| `langton.c` | Turmite | finite state machine |
| `sierpinski.c` | IFS chaos game | affine transforms |
| `fern.c` | Barnsley IFS | 4-transform probability |
| `koch.c` | L-system | recursive subdivision |
| `dragon_curve.c` | L-system | iterative string rewrite |
| `l_system.c` | General L-system | production rules |
| `lissajous.c` | Parametric curves | sin/cos phase |
| `spirograph.c` | Hypotrochoid | parametric gear math |
| `string_art.c` | Modular arithmetic | i → round(i×k) mod N |
| `epicycles.c` | Fourier series | DFT, complex exponentials |
| `automaton_2d.c` | 2D CA | 2D neighborhood rules |
| `hex_life.c` | Hex grid CA | offset-row coordinate mapping |
| `aurora.c` | Sinusoidal curtains | multi-octave sin |
| `cymatics.c` | Chladni figures | 2D standing wave nodes |

---

## Tier 2 — Simulation Basics (2–3 months)
Introduces differential equations and numerical integration.

| File | Subject | Key Math |
|---|---|---|
| `bounce_ball.c` | Newtonian motion | Euler integration |
| `spring_pendulum.c` | Lagrangian mech | Lagrange equations |
| `double_pendulum.c` | Chaos | RK4 integration |
| `pendulum_wave.c` | Phase arithmetic | ω_n = 2π(N+n)/T |
| `lorenz.c` | Strange attractor | RK4, Lyapunov exponents |
| `nbody.c` | Gravity | Velocity Verlet, softening |
| `cloth.c` | Spring-mass | Hooke's law, symplectic Euler |
| `elastic_collision.c` | Billiards | momentum + KE conservation |
| `orbit_3body.c` | Three-body | Verlet, figure-8 IC |
| `gyroscope.c` | Rigid body | Euler angles, torque |

---

## Tier 3 — Fluid and Continuous Systems (3–4 months)
Requires partial differential equations and numerical stability.

| File | Subject | Key Math |
|---|---|---|
| `wave.c` | 1D/2D wave PDE | FDTD, CFL condition |
| `wave_2d.c` | 2D wave PDE | Huygens, interference |
| `reaction_diffusion.c` | Gray-Scott | Laplacian, diffusion |
| `sand.c` | Cellular automaton | simple rules |
| `flowfield.c` | Vector field | fBm Perlin noise |
| `navier_stokes.c` | Stable fluid | Gauss-Seidel, projection, advection |
| `lenia.c` | Continuous GoL | convolution kernel, growth fn |

---

## Tier 4 — Advanced Physics and Math (4–6 months)
Requires linear algebra, complex numbers, statistical mechanics.

| File | Subject | Key Math |
|---|---|---|
| `ising.c` | Stat mechanics | Metropolis MCMC, phase transition |
| `schrodinger.c` | Quantum mech | Crank-Nicolson, Thomas algorithm |
| `orbit_3body.c` | N-body chaos | figure-8 IC, Verlet |
| `mandelbrot.c` | Complex iteration | escape-time, complex plane |
| `julia.c` | Complex dynamics | same as mandelbrot, c fixed |
| `buddhabrot.c` | Orbit density | two-pass sampling, log-normalize |
| `newton_fractal.c` | Complex Newton | Newton-Raphson, basin coloring |
| `strange_attractor.c` | Chaos | Clifford/de Jong map, density accumulation |
| `apollonian.c` | Circle packing | Descartes theorem, recursion |
| `bifurcation.c` | Chaos theory | logistic map, period doubling |
| `burning_ship.c` | Complex fractal | modified Mandelbrot, abs() trick |

---

## Tier 5 — Rendering (parallel with Tier 3–4)
Requires linear algebra and graphics pipeline understanding.

| File | Subject | Key Math |
|---|---|---|
| `wireframe.c` | 3D projection | rotation matrix, perspective |
| `donut.c` | Parametric surface | torus parametrization |
| `raymarcher.c` | SDF ray marching | sphere marching, Blinn-Phong |
| `raymarcher_cube.c` | SDF box | finite-difference normals |
| `raymarcher_primitives.c` | SDF ops | min/max composition |
| `metaballs.c` | Implicit surfaces | summed SDFs |
| `torus_raster.c` | Rasterization | barycentric coords, z-buffer |
| `cube_raster.c` | Shader pipeline | vert/frag shaders, back-face cull |
| `sphere_raster.c` | UV mapping | spherical coordinates |
| `displace_raster.c` | Vertex displacement | central-difference normals |
| `boids_3d.c` | 3D flocking | 3D vectors, perspective projection |
| `penrose.c` | Aperiodic tiling | de Bruijn pentagrid duality |

---

## Tier 6 — Emergent Systems (combine with Tier 5)
Agent-based simulation, graph algorithms, complex systems.

| File | Subject | Key Math |
|---|---|---|
| `flocking.c` | Reynolds boids | separation+alignment+cohesion |
| `shepherd.c` | Herding | flee force, bounded domain |
| `ant_colony.c` | ACO | pheromone decay, stigmergy |
| `voronoi.c` | Voronoi diagram | nearest-neighbor, Langevin |
| `wator.c` | Predator-prey | Wa-Tor rules |
| `graph_search.c` | BFS/DFS/A* | graph traversal |
| `network_sim.c` | Network dynamics | node/edge simulation |
| `reaction_wave.c` | Excitable media | FitzHugh-Nagumo |
| `maze.c` | Graph algorithms | DFS generation, BFS solve |
| `convex_hull.c` | Computational geom | Graham scan, Jarvis march |
| `sort_vis.c` | Algorithms | bubble/insertion/selection/quick/heap |

---

## Terrain and Noise

| File | Subject | Key Math |
|---|---|---|
| `terrain.c` | Diamond-square | fractal subdivision, erosion |
| `perlin_landscape.c` | fBm terrain | Perlin noise, parallax |
| `plasma.c` | Plasma effect | sin-sum palette cycling |

---

## 2-Year Breakdown

| Period | Focus | Milestone |
|---|---|---|
| Month 1–2 | Framework + Tier 1 | Rewrite 5 Tier 1 files from scratch |
| Month 3–5 | Tier 2 | Understand RK4, Verlet; rewrite lorenz from paper |
| Month 6–9 | Tier 3 | Implement Gray-Scott and Navier-Stokes from equations alone |
| Month 10–14 | Tier 4 | Derive Crank-Nicolson; implement Ising from Metropolis paper |
| Month 15–18 | Tier 5 | Build a software rasterizer from scratch |
| Month 19–24 | Combine + Create | Your own topic, your own algorithm |

---

## Books Alongside Each Tier

| Tier | Book |
|---|---|
| 1 | *The Nature of Code* — Daniel Shiffman |
| 2 | *Physics-Based Animation* — Kenny Erleben |
| 3 | *Fluid Simulation for Computer Graphics* — Robert Bridson |
| 4 | *Statistical Mechanics* — Reif; *Quantum Mechanics* — Griffiths |
| 5 | *Real-Time Rendering* — Akenine-Möller |
| 6 | *Complex Adaptive Systems* — Miller & Page |
