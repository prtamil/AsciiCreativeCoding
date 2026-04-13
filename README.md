# ASCII Creative Coding

```
  ██████╗    ███████╗ ██╗ ██╗ ██╗
 ██╔════╝    ██╔════╝ ██║ ██║ ██║
 ██║         █████╗   ██║ ██║ ██║
 ██║         ██╔══╝   ╚═╝ ╚═╝ ╚═╝
 ╚██████╗    ███████╗
  ╚═════╝    ╚══════╝  terminal as canvas
```

100 simulations. Pure C. Zero GUI dependencies. The terminal is the only renderer.

All simulations share a unified architecture and fixed-timestep physics loop.
Each program can be studied independently or as part of the full simulation framework.

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
| `lattice_gas` | FHP-I lattice gas — 6-direction bit-packed hex grid; 64-entry collision lookup (head-on 2-particle + symmetric 3-particle); streaming with bounce-back walls; momentum-colored display; 4 presets (cylinder/double-slit/channel/free), 5 themes |
| `navier_stokes` | Jos Stam stable fluid — Gauss-Seidel diffusion, semi-Lagrangian advection, divergence-free projection |
| `reaction_diffusion` | Gray-Scott model — 7 species presets (Mitosis, Coral, Stripes, Maze…) |
| `lenia` | Continuous Game of Life — smooth kernel convolution, organic moving creatures |
| `wave` | FDTD 2D wave PDE — CFL-stable, 5 interference sources, 4 color themes |
| `wave_2d` | 2D scalar wave PDE — Huygens interference, multiple point sources, signed amplitude colour map |
| `reaction_wave` | FitzHugh-Nagumo excitable medium — activator/inhibitor PDE, spiral waves, 4 color themes |
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
| `ising` | 2D Ising model — Metropolis MCMC spin flips, exp(−ΔE/kT) acceptance, phase transition at T_c |
| `schrodinger` | 1D Schrödinger — Crank-Nicolson tridiagonal (Thomas algorithm), tunneling, 4 presets |
| `blackhole` | Gargantua 3D (Interstellar) — exact Schwarzschild null geodesics via RK4, precomputed lensing table; photon ring from min-radius tracking, primary + secondary disk images, relativistic Doppler beaming D=[(1+β)/(1−β)]^1.5, gravitational redshift; dynamic clip radius scales with cam_dist; 11 themes; `+/-` zoom |
| `magnetic_field` | 2D dipole field lines — Biot-Savart superposition of magnetic monopoles, RK4 streamline tracing from N-pole seeds, 4 presets (Dipole/Quadrupole/Attract/Repel), incremental reveal animation, 5 themes |
| `chain` | Hanging chain & swinging rope — Position-Based Dynamics (Verlet + iterative distance-constraint projection), tension-coloured links, 4 presets (Hanging/Pendulum/Bridge/Wave), trail ring-buffer, 5 themes |
| `rigid_body` | 2D rigid body physics — cubes + spheres with AABB collision; cube-cube (min-axis overlap), sphere-sphere (distance), cube-sphere (closest-point on AABB); unified impulse resolver `j=(1+e)·vn/(imA+imB)`, Coulomb friction, Baumgarte positional correction, sleep-counter resting; `c` add cube, `s` add sphere, `x` remove last, `r` reset |
| `soft_body` | Jelly blob — 7×7 spring-mass mesh; structural + shear + bending springs (Hooke + velocity damping); Newtonian pressure from shoelace area vs target; scan-line fill rendering; symplectic Euler integration; 4 presets (Blob/Heavy/Bouncy/Two), 5 themes |

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

### Analytic Ray Tracing
| Program | Algorithm |
|---------|-----------|
| `sphere_raytrace` | Quadratic ray-sphere — orbiting camera, 3-point Phong, Fresnel glass mode, 256-color, 6 themes |
| `cube_raytrace` | AABB slab method — inverse-rotation ray transform, face-normal colour, pixel-perfect wireframe, 6 themes |
| `torus_raytrace` | Quartic intersection (sampling + bisection) — ring in XZ plane, gradient normal, Fresnel, 6 themes |
| `capsule_raytrace` | Cylinder + hemisphere caps — axial projection body normal, cap sphere normal, inverse-rotation transform, 6 themes |

### Emergent Systems
| Program | Algorithm |
|---------|-----------|
| `flocking` | Reynolds boids — 5 modes (classic/leader/Vicsek/orbit/predator-prey) |
| `shepherd` | User-controlled herding — flee force, panic zone, flee-radius ring |
| `ant_colony` | Pheromone ACO — stigmergic path optimization |
| `wator` | Wa-Tor predator-prey ecosystem |
| `network_sim` | SIR epidemic + spring-force graph layout |
| `slime_mold` | Physarum polycephalum — Jeff Jones (2010) agent model; 3-sensor sense→rotate→move→deposit loop; double-buffered trail diffusion + decay; emergent minimum Steiner tree networks connecting food sources; 4 presets, 5 themes |

### Geometry
| Program | Algorithm |
|---------|-----------|
| `rect_grid` | Rectangular character grid — per-cell random rate, dual sinusoidal colour waves, 6 themes |
| `polar_grid` | Polar character grid — concentric rings, alternating rotation, colour by ring, 6 themes |
| `hex_grid` | Hexagonal character grid — offset-row tiling, cube-coordinate ring distance for colour bands |
| `lissajous` | Harmonograph/Lissajous — two damped perpendicular oscillators, phase drift |
| `spirograph` | Hypotrochoid parametric curves — 3 simultaneous with parameter drift |
| `string_art` | Modular arithmetic i→⌊i×k⌋ mod N, morphing cardioid/nephroid/astroid |
| `voronoi` | Brute-force nearest-neighbor, Langevin seed motion, d2−d1 edge detection |
| `convex_hull` | Graham scan + Jarvis march — simultaneous race |

### Mathematical Art
| Program | Algorithm |
|---------|-----------|
| `epicycles` | DFT epicycles — sorted-by-amplitude arm chain, convergence animation |
| `fourier_art` | User-drawn path → Fourier reconstruction — draw any shape with cursor keys, arc-length resample to 256 pts, O(N²) DFT, epicycle arm chain replay with auto-add convergence, 5 themes |
| `cymatics` | Chladni figures — 2D standing wave nodal lines, 20 modes |
| `plasma` | Demoscene: 4-component sin-sum, palette cycling |
| `aurora` | Multi-octave sinusoidal curtains, deterministic star hash |
| `penrose` | de Bruijn pentagrid duality — aperiodic tiling, slow rotation |
| `terrain` | Diamond-square heightmap — thermal erosion, 7 contour levels |
| `perlin_landscape` | Perlin fBm — 3 parallax terrain layers, 5-octave noise, painter's algorithm |

### Artistic / Biological
| Program | Algorithm |
|---------|-----------|
| `galaxy` | Spiral galaxy — 3000 stars in circular orbits with flat rotation curve (ω = v₀/r); logarithmic spiral arm initialization; brightness accumulator grid with per-frame decay creates natural trails; normalised density → char (`.,:oO0@`); radial colour zones (core/disk/halo); 2–4 arms, 5 themes |
| `jellyfish` | Physics pulse locomotion — IDLE sink → CONTRACT jet → GLIDE coast → EXPAND bloom; asymmetric bell (width × height axes); tentacle inertia lag |
| `xrayswarm` | Multi-swarm radial pulse — DIVERGE → PAUSE → CONVERGE; workers park at screen edge, retrace exact origin path; 4-pass rendering prevents trail cancellation |
| `gear` | Wireframe rotating gear — proximity-based edge detection, tangential surface-velocity sparks, speed-proportional emission, 10 color themes (fire/matrix/plasma/nova/poison/ocean/gold/neon/arctic/lava) |
| `railwaymap` | Procedural transit map — H/V/Z grid-aligned line templates, canvas-based ACS junction detection, station interchange, 10 themes |
| `fireworks_rain` | Fireworks with matrix-rain arc trails — each of 72 sparks per explosion grows a 16-slot position-history trail; chars shimmer 75 % per tick; 5 themes (vivid/matrix/fire/ice/plasma) remap all spark color pairs; `t` cycles theme |
| `matrix_snowflake` | Matrix rain + live DLA snowflake — two real simulations on one screen: classic digital rain in the background; a D6-symmetric DLA ice crystal grows from the center in the foreground, freezing 12 symmetric positions per walker stick event; crystal flashes white on completion then resets; 5 themes (Classic/Inferno/Nebula/Toxic/Gold) |

### Algorithms
| Program | Algorithm |
|---------|-----------|
| `sort_vis` | 5 sort algorithms — animated bar chart, comparison+swap counters |
| `maze` | DFS generation + BFS/A* animated solve |
| `graph_search` | BFS/DFS/A* on grid — animated frontier expansion |
| `forest_fire` | Drossel-Schwabl CA — 3-state (EMPTY/TREE/FIRE) probabilistic update; neighbour-spread + lightning ignition; ratio p/f controls cluster size and self-organised criticality; 4-way/8-way spread, 4 presets, 5 themes |

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
gcc -std=c11 -O2 -Wall -Wextra physics/schrodinger.c       -o schrodinger    -lncurses -lm
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
├── fluid/             — Navier-Stokes, Gray-Scott, wave PDE, FitzHugh-Nagumo, Lenia
├── flocking/          — Reynolds boids, shepherd herding
├── fractal_random/    — Mandelbrot, Julia, Newton, Apollonian, terrain, Perlin landscape
├── geometry/          — parametric curves, grids, computational geometry (lissajous, voronoi, convex hull…)
├── matrix_rain/       — Matrix rain variants (classic rain, DLA snowflake hybrid)
├── misc/              — sorting, maze, CA music
├── particle_systems/  — fire, fireworks, explosions
├── physics/           — Lorenz, N-body, cloth, pendulums, Ising, Schrödinger, Schwarzschild black hole
├── raster/            — software rasterizer (torus, cube, sphere)
├── raymarcher/        — SDF ray marching
├── raytracing/        — analytic ray tracing (sphere, cube, torus, capsule)
├── ncurses_basics/    — framework reference implementations
└── documentation/
    ├── Claude.md          — complete build reference
    └── learning/
        ├── ROADMAP.md         — 6-tier study order, 2-year plan
        └── concept_*.md       — 96 deep-dive concept files
                                 (math → pseudocode → implementation notes)
```

---

## Documentation

`documentation/learning/` contains 97 concept files — one per program. Each file has two passes:

- **Pass 1** — core idea, mental model, key equations, data structures, non-obvious design decisions, open questions to explore
- **Pass 2** — pseudocode, module map, data flow diagram, core loop

`documentation/learning/ROADMAP.md` gives the optimal study order across 6 tiers (visual math → simulation → fluid PDE → advanced physics → rendering → emergent systems) with a 2-year breakdown and a book recommendation per tier.

---

## Dependencies

- `gcc` (C11)
- `libncurses` (`sudo apt install libncurses5-dev` or equivalent)
- `libm` (standard, always available)

Nothing else. No CMake, no package manager, no runtime dependencies.
