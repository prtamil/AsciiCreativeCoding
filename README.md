# ASCII Creative Coding

```
 ╔══════════════════════════════════════════════════════════════════════╗
 ║                                                                      ║
 ║   ██╗   ██╗ ███████╗ ███████╗ ██╗      ███████╗ ███████╗ ███████╗    ║
 ║   ██║   ██║ ██╔════╝ ██╔════╝ ██║      ██╔════╝ ██╔════╝ ██╔════╝    ║
 ║   ██║   ██║ ███████╗ █████╗   ██║      █████╗   ███████╗ ███████╗    ║
 ║   ██║   ██║ ╚════██║ ██╔══╝   ██║      ██╔══╝   ╚════██║ ╚════██║    ║
 ║   ╚██████╔╝ ███████║ ███████╗ ███████╗ ███████╗ ███████║ ███████║    ║
 ║    ╚═════╝  ╚══════╝ ╚══════╝ ╚══════╝ ╚══════╝ ╚══════╝ ╚══════╝    ║
 ║                                                                      ║
 ║                          useless projects                            ║
 ║                                                                      ║
 ║       the most useful thing in the world is being useless            ║
 ║                                                                      ║
 ╚══════════════════════════════════════════════════════════════════════╝
```

**327 programs. Pure C. ncurses. No GUI. Each file complete in itself.**

---

## Overview

> *"All art is quite useless."* — Oscar Wilde

---

## What This Is

A collection of real-time interactive simulations built entirely in C with
ncurses. Every program runs in a terminal window — no OpenGL, no SDL, no
graphics library. Forcing complex physics and rendering through a character
grid sharpens the understanding of every algorithm involved.

Topics span from elementary cellular automata to the Navier-Stokes equations.
From Conway's Game of Life to a Crank-Nicolson Schrödinger solver. From a
Bresenham wireframe to a full SDF raymarcher with Blinn-Phong shading to a
Cornell-box Monte Carlo path tracer.

**Build requirement:** `gcc`, `ncurses`, `libm`. Nothing else.

Per-program algorithm notes: [DEMOS.md](DEMOS.md). The `grids/` folder
carries its own [README](grids/README.md) covering the four tiling families
and their reading order.

---

## Design Choices

**Every file is self-contained — by intention, not by accident.**
Self-containment is the goal, the explicit anti-thesis to code reuse. Forbidding
shared helpers forces re-deriving every primitive in the file that needs it.
Even a trivial `clamp()` or `lerp()` gets read, audited, and tuned in the
specific context where it's used. The point is to own every line — to optimise
and understand even the trivial functions, rather than treat them as solved.

**Plain C — `struct`s and functions only.**
C is a deliberate choice, not the only available one. Rust, C++, and Clojure
are in the toolbox; C is what gets reached for when the goal is to see the
algorithm with nothing standing between it and the reader. The hardcore-C
ethos here is taken seriously: no classes, no templates, no macros-as-DSL.

**Copying is the intended workflow.**
```bash
gcc filename.c -lncurses -lm && ./a.out
```
That is the workflow. No build system, no CMake, no Makefile, no package
manager. A single file is a single program.

**POSIX terminal only.**
No Windows, no GUI. Forcing a Navier-Stokes solver or a path tracer through
a character grid demands a sharper understanding of the underlying math
than reaching for a graphics API would. The terminal is not a limitation —
it is the whole point.

**One physics model, one rendering model, applied uniformly.**
Every file uses the same fixed-timestep accumulator, the same pixel-space
coordinate model, and the same ncurses double-buffer sequence. Read one
file, you can read any other. The framework is not hidden — it's the first
thing documented in every source file. See [CLAUDE.md](CLAUDE.md) for the
full convention.

**Documentation as a first-class artefact.**
Every source file opens with a CONCEPTS and MENTAL MODEL block; the
`documentation/` directory carries long-form essays on the algorithms,
visual techniques, and ncurses idioms used throughout. The project doubles
as a self-paced curriculum — the reading material is the point as much as
the code is.

**AI-assisted, human-directed.**
This project exists at this scale because of AI tooling, and that is worth
naming directly. What preceded it was scattered interests, partial demos,
half-finished goals, and a backlog of curiosity that normally ends up
abandoned. AI provided the leverage to organise the chaos and materialise
it into 327 working programs. The direction, taste, architectural
conventions, and pedagogical structure are mine; the throughput is what the
collaboration made possible.

---

## Demos

Every program lives in a topic folder. Folders summarised here;
per-program algorithm notes in [DEMOS.md](DEMOS.md).

| Folder | Files | Summary |
|--------|------:|---------|
| `fluid/` | 11 | Stam stable fluids, lattice Boltzmann, FDTD wave, Gray-Scott reaction-diffusion, FitzHugh-Nagumo excitable medium, SPH, falling sand, vorticity-streamfunction, CFL stability explorer |
| `physics/` | 36 | Lorenz / N-body / cloth / Ising / Schrödinger; Schwarzschild black hole; quaternion gyroscope; PBD chains; rigid-body, soft-body; Barnes-Hut O(N log N) gravity; mass-spring lattice; CG and multigrid visualisers; RK1/2/4 comparison |
| `procedural/` | 69 | Six sub-folders: chaos, fields, fractals, generational, patterns, worldgen — Mandelbrot / Julia / Buddhabrot / Newton; Barnsley IFS; DLA; Lyapunov; logistic; Apollonian; L-systems; Lorenz; Truchet/Wang/quasicrystal/Penrose; star fields, galaxies, hydraulic erosion, fBm clouds |
| `grids/` | 76 | Four families (rect/hex/tri/polar) × *drawing* + *placement*. Triangular covers regular tilings (1–6), recursive fractals (7–9), aperiodic substitution (10, 12), and Delaunay (11). See [grids/README.md](grids/README.md) |
| `raster/` | 14 | Software rasteriser: cube/sphere/torus, deferred pipeline, shadow mapping, SSAO, bloom, neon edges, marching cubes, Mandelbulb raster, the spinning ASCII donut |
| `raymarcher/` | 8 | Sphere tracing on SDFs: primitives, CSG atlas, blend/twist/repeat composition gallery, metaballs, KIFS fractal, Mandelbulb |
| `raytracing/` | 10 | Analytic ray↔primitive: sphere/cube/torus/capsule; tunnel, forest god rays, solar eclipse, ringed Saturn; Cornell-box Monte Carlo path tracer |
| `artistic/` | 23 | Aesthetic effects: galaxy, aurora, mandalas, plasma, Hindu / Islamic geometric patterns, Penrose pentagrid, cymatics, transit map, bonsai gallery, jellyfish, DNA helix |
| `animation/` | 14 | Forward + inverse kinematics, ragdoll, Verlet ropes, easing curves, snake / centipede / tentacle / medusa |
| `flocking/` | 7 | Reynolds boids, shepherd herding, crowd steering, faction battle, ant-colony pheromone, predator-prey, Physarum slime mould |
| `robots/` | 4 | Hexapod tripod gait, biped, spring-leg jumper, self-balancing Perlin-terrain bot |
| `algorithms/` | 11 | Quadtree / k-d / BSP; convex hull, Voronoi, visibility; BFS/DFS/A\*; sort visualiser; SIR epidemic |
| `geometry/` | 4 | Lissajous, spirograph, string-art, Delaunay |
| `signal/` | 10 | DFT, FFT, IDFT, convolution, sampling, time-domain filters, 2-D Fourier |
| `particle_systems/` | 18 | Fire (3 algos), smoke, fireworks, kaboom shockwave, fountain, comet, rain, snow, constellation network |
| `matrix_rain/` | 5 | Classic Matrix rain + snowflake / pulsar / sun-mask / freeze variants |
| `Ai/` | 2 | Genetic Smart Rockets, feed-forward neural-net visualiser |
| `turtle/` | 1 | Logo-style turtle polygon animator |
| `ncurses_basics/` | 4 | Framework reference programs |

---

## Build

```bash
# Universal pattern — same shape for every file in the repo:
gcc -std=c11 -O2 -Wall -Wextra <folder>/<file>.c -o <name> -lncurses -lm

# Examples:
gcc -std=c11 -O2 -Wall -Wextra fluid/navier_stokes.c           -o navier_stokes -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/lorenz.c                -o lorenz        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra procedural/fractals/mandelbrot.c -o mandelbrot   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher.c         -o raymarcher    -lncurses -lm
```

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
| `Space` | trigger event / advance state |

---

## Documentation

- [CLAUDE.md](CLAUDE.md) — full project conventions: section maps, comment
  standards, CONCEPTS / MENTAL MODEL block format, build rules
- [DEMOS.md](DEMOS.md) — per-program algorithm notes
- [documentation/Architecture.md](documentation/Architecture.md) — framework
  design, loop mechanics, coordinate model, per-subsystem deep dives
- [documentation/Master.md](documentation/Master.md) — every technique used
  in the project explained from first principles, with source files and
  canonical references
- [documentation/Visual.md](documentation/Visual.md) — ncurses field guide
- [documentation/COLOR.md](documentation/COLOR.md) — palettes, escape-time
  colouring, 256-colour patterns
- [grids/README.md](grids/README.md) — the four tiling families and their
  reading order (the one folder with its own walkthrough)

---

## Dependencies

- `gcc` (C11)
- `libncurses` (`sudo apt install libncurses5-dev` or equivalent)
- `libm` (standard)

Nothing else. No CMake, no package manager, no runtime dependencies.
