# ASCII Creative Coding

```
  ██████╗    ███████╗ ██╗ ██╗ ██╗
 ██╔════╝    ██╔════╝ ██║ ██║ ██║
 ██║         █████╗   ██║ ██║ ██║
 ██║         ██╔══╝   ╚═╝ ╚═╝ ╚═╝
 ╚██████╗    ███████╗
  ╚═════╝    ╚══════╝  terminal as canvas
```

91 simulations. Pure C. Zero GUI dependencies. The terminal is the only renderer.

---

## What This Is

A collection of real-time interactive simulations built entirely in C with ncurses. Every program runs in a terminal window — no OpenGL, no SDL, no graphics library. The constraint is the point: forcing complex physics and rendering through a character grid sharpens the understanding of every algorithm involved.

Topics span from elementary cellular automata to the Navier-Stokes equations. From Conway's Game of Life to a Crank-Nicolson Schrödinger solver. From a Bresenham wireframe renderer to a full SDF raymarcher with Blinn-Phong shading.

**Build requirement:** `gcc`, `ncurses`, `libm`. That's it.

---

## Demos

### Fluid Dynamics
| Program | Algorithm |
|---------|-----------|
| `navier_stokes` | Jos Stam stable fluid — Gauss-Seidel diffusion, semi-Lagrangian advection, divergence-free projection |
| `reaction_diffusion` | Gray-Scott model — 7 species presets (Mitosis, Coral, Stripes, Maze…) |
| `lenia` | Continuous Game of Life — smooth kernel convolution, organic moving creatures |
| `wave` | FDTD 2D wave PDE — CFL-stable, 5 interference sources, 4 color themes |
| `flowfield` | Perlin fBm vector field — bilinear sampling, 8-direction particle trails |

### Physics
| Program | Algorithm |
|---------|-----------|
| `lorenz` | RK4 integration — rotating 3D projection, Lyapunov ghost trajectory |
| `double_pendulum` | Chaos via RK4 — 500-slot trail, real-time divergence metric |
| `nbody` | Velocity Verlet gravity — 20 bodies, softened 1/r², optional black hole |
| `cloth` | Spring-mass network — Hooke's law, symplectic Euler, 3 boundary modes |
| `orbit_3body` | Figure-8 stable orbit → chaos on perturbation |
| `pendulum_wave` | 15 harmonic pendulums — analytic ω_n, re-sync at T=60s |
| `elastic_collision` | Hard-sphere billiards — Maxwell-Boltzmann distribution emerges |
| `schrodinger` | 1D Schrödinger — Crank-Nicolson tridiagonal (Thomas algorithm), tunneling |

### Fractals & Chaos
| Program | Algorithm |
|---------|-----------|
| `mandelbrot` / `julia` | Escape-time iteration, Fisher-Yates random reveal, 6 zoom presets |
| `buddhabrot` | Two-pass orbit density accumulation, log-normalized nebula palette |
| `newton_fractal` | Complex Newton-Raphson on z⁴−1, basin coloring |
| `strange_attractor` | Clifford/de Jong/Ikeda density maps, log-normalized hit grid |
| `burning_ship` | Modified Mandelbrot with abs() trick |
| `bifurcation` | Logistic map, Feigenbaum period-doubling route to chaos |
| `apollonian` | Descartes' circle theorem, recursive circle packing |
| `lorenz` | Strange attractor — RK4, rotating projection |

### Cellular Automata & Life
| Program | Algorithm |
|---------|-----------|
| `life` | Conway GoL + 5 rule variants (HighLife/Seeds/Day&Night…) |
| `cellular_automata_1d` | Wolfram 256 rules, 5 complexity classes color-coded |
| `langton` | Langton's ant + 7 multi-color turmite variants |
| `lenia` | Continuous CA — smooth kernel, real organisms |
| `hex_life` | Game of Life on hexagonal grid (6-neighbor offset layout) |
| `automaton_2d` | General 2D outer-totalistic CA with rule editing |

### Rendering & 3D
| Program | Algorithm |
|---------|-----------|
| `raymarcher` | Sphere-marching SDF — Blinn-Phong, gamma correction |
| `raymarcher_cube` | SDF box — finite-difference normals, shadow ray |
| `raymarcher_primitives` | SDF boolean composition (min/max) — sphere/box/torus/capsule/cone |
| `torus_raster` | UV rasterizer — Phong/toon/normal/wireframe shaders, back-face cull |
| `cube_raster` / `sphere_raster` | Full software rasterizer pipeline |
| `displace_raster` | Real-time vertex displacement, central-difference normal recompute |
| `donut` | Parametric torus projection — the original spinning donut |
| `wireframe` | 3D Bresenham edge projection, slope-to-character line drawing |
| `boids_3d` | Reynolds flocking in 3D — perspective projection, depth-cued characters |

### Emergent Systems
| Program | Algorithm |
|---------|-----------|
| `flocking` | Reynolds boids — 5 modes (classic/leader/Vicsek/orbit/predator-prey) |
| `shepherd` | User-controlled herding — flee force, panic zone, flee-radius ring |
| `ant_colony` | Pheromone ACO — stigmergic path optimization |
| `voronoi` | Brute-force nearest-neighbor, Langevin seed motion, d2−d1 edge detection |
| `wator` | Wa-Tor predator-prey ecosystem |
| `network_sim` | SIR epidemic + spring-force graph layout |

### Statistical Mechanics & Quantum
| Program | Algorithm |
|---------|-----------|
| `ising` | Metropolis MCMC — precomputed acceptance table, phase transition at T_c |
| `schrodinger` | Crank-Nicolson — unitary evolution, Gaussian wave packet, quantum tunneling |

### Mathematical Art
| Program | Algorithm |
|---------|-----------|
| `epicycles` | DFT epicycles — sorted-by-amplitude arm chain, convergence animation |
| `spirograph` | Hypotrochoid parametric curves — 3 simultaneous with parameter drift |
| `lissajous` | Phase-locked sin/cos figures |
| `string_art` | Modular arithmetic i→⌊i×k⌋ mod N, morphing cardioid/nephroid/astroid |
| `cymatics` | Chladni figures — 2D standing wave nodal lines, 20 modes |
| `plasma` | Demoscene: 4-component sin-sum, palette cycling |
| `aurora` | Multi-octave sinusoidal curtains, deterministic star hash |
| `penrose` | de Bruijn pentagrid duality — aperiodic tiling, slow rotation |
| `terrain` | Diamond-square heightmap — thermal erosion, 7 contour levels |

### Artistic / Biological
| Program | Algorithm |
|---------|-----------|
| `jellyfish` | Physics pulse locomotion — IDLE sink → CONTRACT jet → GLIDE coast → EXPAND bloom; asymmetric bell (width × height axes); tentacle inertia lag |
| `xrayswarm` | Multi-swarm radial pulse — DIVERGE → PAUSE → CONVERGE; workers park at screen edge, retrace exact origin path; 4-pass rendering prevents trail cancellation |

### Algorithms
| Program | Algorithm |
|---------|-----------|
| `sort_vis` | 5 sort algorithms — animated bar chart, comparison+swap counters |
| `maze` | DFS generation + BFS/A* animated solve |
| `convex_hull` | Graham scan + Jarvis march — simultaneous race |
| `graph_search` | BFS/DFS/A* on grid — animated frontier expansion |

---

## Architecture

Every simulation uses the same framework:

```
§1 config   — all constants in one place
§2 clock    — CLOCK_MONOTONIC nanosecond timer
§3 color    — 256-color with 8-color fallback
§4 physics  — simulation state, fixed-timestep step()
§5 draw     — scene_draw() via ncurses primitives
§6 app      — main loop: input → physics → render → sleep
```

Physics runs in **pixel space** (`CELL_W=8 px`, `CELL_H=16 px` per terminal cell), independent of terminal size. The only coordinate conversion is inside `scene_draw()`.

Frame sequence: `erase() → draw → wnoutrefresh() → doupdate()`. The `typeahead(-1)` call prevents tearing. No custom double-buffer — ncurses provides it.

---

## Build

```bash
# Any single program — same pattern everywhere:
gcc -std=c11 -O2 -Wall -Wextra <folder>/<file>.c -o <name> -lncurses -lm

# Examples:
gcc -std=c11 -O2 -Wall -Wextra fluid/navier_stokes.c      -o navier_stokes  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/lorenz.c            -o lorenz         -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/mandelbrot.c -o mandelbrot     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher.c     -o raymarcher     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra misc/schrodinger.c          -o schrodinger    -lncurses -lm
```

See `Claude.md` for the complete build list.

---

## Keys (Common)

| Key | Action |
|-----|--------|
| `q` / `ESC` | quit |
| `p` | pause / resume |
| `r` | reset |
| `+` / `-` | increase / decrease primary parameter |
| Arrow keys | move / steer (where applicable) |
| `1`–`5` | switch preset / mode |
| `Space` | trigger event or jump to next state |

---

## Structure

```
.
├── artistic/          — parametric art, CA, L-systems, visual math
├── fluid/             — Navier-Stokes, Gray-Scott, wave PDE, Lenia
├── flocking/          — Reynolds boids, shepherd herding
├── fractal_random/    — Mandelbrot, Julia, Newton, Apollonian, terrain
├── matrix_rain/       — Matrix rain
├── misc/              — Schrödinger, Ising, sorting, maze, Perlin
├── particle_systems/  — fire, fireworks, explosions
├── physics/           — Lorenz, N-body, cloth, pendulums
├── raster/            — software rasterizer (torus, cube, sphere)
├── raymarcher/        — SDF ray marching
├── ncurses_basics/    — framework reference implementations
└── documentation/
    ├── Claude.md          — complete build reference
    └── learning/
        ├── ROADMAP.md         — 6-tier study order, 2-year plan
        └── concept_*.md       — 89 deep-dive concept files
                                 (math → pseudocode → implementation notes)
```

---

## Documentation

`documentation/learning/` contains 91 concept files — one per program. Each file has two passes:

- **Pass 1** — core idea, mental model, key equations, data structures, non-obvious design decisions, open questions to explore
- **Pass 2** — pseudocode, module map, data flow diagram, core loop

`documentation/learning/ROADMAP.md` gives the optimal study order across 6 tiers (visual math → simulation → fluid PDE → advanced physics → rendering → emergent systems) with a 2-year breakdown and a book recommendation per tier.

---

## Dependencies

- `gcc` (C11)
- `libncurses` (`sudo apt install libncurses5-dev` or equivalent)
- `libm` (standard, always available)

Nothing else. No CMake, no package manager, no runtime dependencies.
