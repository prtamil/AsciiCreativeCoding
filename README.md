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

A unified architecture and a fixed-timestep physics loop run underneath all of
them; every program can be read standalone or as part of the larger framework.

---

## Overview

> *"Everyone knows the use of the useful, but nobody knows the use of the useless."*
> — Zhuangzi

The useful arithmetic — mortgages, deadlines, dependents — is what keeps me
afloat, and I respect it. This project is the *other* arithmetic. None of
these programs solves a problem, ships a feature, or earns a thing. They
exist so I can spend some private hours where the only stake is whether a
number on the screen matches my idea of what beauty looks like.

> *"All art is quite useless."* — Oscar Wilde

And here, deliberately, I break all the production-code rules. In day-job
code, every interface widens with edge cases until the abstraction survives
every imaginable caller; every helper sits in another file three folders
away; every level of indirection is another contract to hold in your head.
By the time you've absorbed the framework, the hour is gone — and so is
whatever spark made you open the editor.

So here, the opposite. **One file is one program.** Each program is
specifically designed for the one problem in front of me — no more, no less.
No future-proofing, no configurable knobs for cases that may never come, no
"we might need this someday" hooks. The duplication is the whole point. The
freedom is the whole point.

> *"How we spend our days is, of course, how we spend our lives."*
> — Annie Dillard, *The Writing Life*

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

Per-program algorithm notes: [DEMOS.md](DEMOS.md). Folder-level overviews:
each completed folder has its own `README.md` (e.g. [grids/README.md](grids/README.md),
[raster/README.md](raster/README.md), [raymarcher/README.md](raymarcher/README.md)).

---

## Design Choices

> *"Perfection is achieved, not when there is nothing more to add, but
>  when there is nothing left to take away."*
> — Antoine de Saint-Exupéry, *Terre des Hommes*

This is not a library, not a framework, not a toolkit. It is a sketchbook —
327 individual programs, each complete in itself, each existing for no
reason other than that it is interesting to build and beautiful to watch.

**Every file is self-contained by intention — a manual choice, not a
build-system default.**
No shared headers, no common modules, no inter-file dependencies. The rule
is set on purpose and preserved on purpose; nothing about C or ncurses
required it. After enough years of jumping between fifteen open tabs to
track where one constant is defined, which helper actually mutates state,
how one edge case sneaks in from another module — I came to believe the
search itself is the cost most codebases pretend doesn't exist. When I sit
down to **learn** how a program works, or to **optimise** one knob inside
it, I want every line that matters in the file I already have open. There
is no jumping between files here, because there is nothing in another file
to jump to.

Code duplication is the deliberate trade-off. A repeated 20-line physics
loop in every file is better than a shared abstraction that requires
understanding six other files before you can touch one. If a second program
needs the same loop, it contains the same loop — sometimes with its own
switch-case in the middle, because *this* program has an edge case the
others don't. The duplication is the freedom for each file to be **complete
and specific to the program it is**. The shared concept stays shared in
your head, not in a header. Change one simulation, only one file changes.
No edge cases bleed across boundaries. Delete any file and nothing else
breaks.

**Plain C — `struct`s and functions only.**
No classes, no inheritance, no module system. The cost is a little
duplication; the benefit is that any program can be read top-to-bottom in a
single sitting, and every change stays exactly where you put it.

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

---

## Demos

Every program lives in a topic folder. Folders summarised here; per-program
algorithm notes in [DEMOS.md](DEMOS.md); folder-level READMEs (where
present) give the reading order and the unifying primitive for that folder.

| Folder | Files | Summary |
|--------|------:|---------|
| `fluid/` | 11 | Stam stable fluids, lattice Boltzmann, FDTD wave, Gray-Scott reaction-diffusion, FitzHugh-Nagumo excitable medium, SPH, falling sand, vorticity-streamfunction, CFL stability explorer |
| `physics/` | 36 | Lorenz / N-body / cloth / Ising / Schrödinger; Schwarzschild black hole; quaternion gyroscope; PBD chains; rigid-body, soft-body; Barnes-Hut O(N log N) gravity; mass-spring lattice; CG and multigrid visualisers; RK1/2/4 comparison |
| `procedural/` | 69 | Six sub-folders: chaos, fields, fractals, generational, patterns, worldgen — Mandelbrot / Julia / Buddhabrot / Newton; Barnsley IFS; DLA; Lyapunov; logistic; Apollonian; L-systems; Lorenz; Truchet/Wang/quasicrystal/Penrose; star fields, galaxies, hydraulic erosion, fBm clouds |
| `grids/` | 76 | Four families (rect/hex/tri/polar) × *drawing* + *placement*. Triangular covers regular tilings (1–6), recursive fractals (7–9), aperiodic substitution (10, 12), and Delaunay (11). See [grids/README.md](grids/README.md) |
| `raster/` | 14 | Software rasteriser: cube/sphere/torus, deferred pipeline, shadow mapping, SSAO, bloom, neon edges, marching cubes, Mandelbulb raster, the spinning ASCII donut. See [raster/README.md](raster/README.md) |
| `raymarcher/` | 8 | Sphere tracing on SDFs: primitives, CSG atlas, blend/twist/repeat composition gallery, metaballs, KIFS fractal, Mandelbulb. See [raymarcher/README.md](raymarcher/README.md) |
| `raytracing/` | 10 | Analytic ray↔primitive: sphere/cube/torus/capsule; tunnel, forest god rays, solar eclipse, ringed Saturn; Cornell-box Monte Carlo path tracer. See [raytracing/README.md](raytracing/README.md) |
| `artistic/` | 23 | Aesthetic effects: galaxy, aurora, mandalas, plasma, Hindu / Islamic geometric patterns, Penrose pentagrid, cymatics, transit map, bonsai gallery, jellyfish, DNA helix |
| `animation/` | 14 | Forward + inverse kinematics, ragdoll, Verlet ropes, easing curves, snake / centipede / tentacle / medusa. See [animation/README.md](animation/README.md) |
| `flocking/` | 7 | Reynolds boids, shepherd herding, crowd steering, faction battle, ant-colony pheromone, predator-prey, Physarum slime mould. See [flocking/README.md](flocking/README.md) |
| `robots/` | 4 | Hexapod tripod gait, biped, spring-leg jumper, self-balancing Perlin-terrain bot. See [robots/README.md](robots/README.md) |
| `algorithms/` | 11 | Quadtree / k-d / BSP; convex hull, Voronoi, visibility; BFS/DFS/A\*; sort visualiser; SIR epidemic. See [algorithms/README.md](algorithms/README.md) |
| `geometry/` | 4 | Lissajous, spirograph, string-art, Delaunay |
| `signal/` | 10 | DFT, FFT, IDFT, convolution, sampling, time-domain filters, 2-D Fourier. See [signal/README.md](signal/README.md) |
| `particle_systems/` | 18 | Fire (3 algos), smoke, fireworks, kaboom shockwave, fountain, comet, rain, snow, constellation network |
| `matrix_rain/` | 5 | Classic Matrix rain + snowflake / pulsar / sun-mask / freeze variants. See [matrix_rain/README.md](matrix_rain/README.md) |
| `Ai/` | 2 | Genetic Smart Rockets, feed-forward neural-net visualiser. See [Ai/README.md](Ai/README.md) |
| `turtle/` | 1 | Logo-style turtle polygon animator. See [turtle/README.md](turtle/README.md) |
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
- Folder-level READMEs: `Ai/`, `algorithms/`, `animation/`, `flocking/`,
  `fluid/`, `grids/`, `matrix_rain/`, `raster/`, `raymarcher/`,
  `raytracing/`, `robots/`, `signal/`, `turtle/`

---

## Dependencies

- `gcc` (C11)
- `libncurses` (`sudo apt install libncurses5-dev` or equivalent)
- `libm` (standard)

Nothing else. No CMake, no package manager, no runtime dependencies.
