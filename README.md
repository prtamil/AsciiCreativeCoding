# ASCII Creative Coding

```
███╗   ███╗ ████████╗  ██████╗   █████╗        .:+##@@@@@@@@@@@##+ :.
████╗ ████║ ╚══██╔══╝ ██╔════╝  ██╔══██╗     +#@@@#:.          :#@@@+
██╔████╔██║    ██║    ██║  ███╗  ███████║    #@@@#.  @@@@@@@@@@  .#@@@#
██║╚██╔╝██║    ██║    ██║   ██║  ██╔══██║     +#@@@#:.          :#@@@+
██║ ╚═╝ ██║    ██║    ╚██████╔╝  ██║  ██║       .:+##@@@@@@@@@@@##+ :.
╚═╝     ╚═╝    ╚═╝     ╚═════╝   ╚═╝  ╚═╝   Make Terminal Great Again
```

257 programs. Pure C. Zero GUI dependencies. MTGA — Make Terminal Great Again.

All simulations share a unified architecture and fixed-timestep physics loop.
Each program can be studied independently or as part of the full simulation framework.

---

## What This Is

A collection of real-time interactive simulations built entirely in C with ncurses. Every program runs in a terminal window — no OpenGL, no SDL, no graphics library. The constraint is the point: forcing complex physics and rendering through a character grid sharpens the understanding of every algorithm involved.

Topics span from elementary cellular automata to the Navier-Stokes equations. From Conway's Game of Life to a Crank-Nicolson Schrödinger solver. From a Bresenham wireframe renderer to a full SDF raymarcher with Blinn-Phong shading.

**Build requirement:** `gcc`, `ncurses`, `libm`. That's it.

For per-program algorithm notes, see [DEMOS.md](DEMOS.md).

---

## Design Choices

> *"All art is quite useless."*
> — Oscar Wilde, The Picture of Dorian Gray

This project is not a library. It is not a framework. It is not a toolkit.
It is closer to a sketchbook — 257 individual programs, each complete in itself,
each existing for no reason other than that it is interesting to build and beautiful to watch.

**Every file is self-contained by intention.**
There are no shared headers, no common modules, no inter-file dependencies. Code
duplication is a deliberate trade-off: a repeated 20-line physics loop in every
file is far better than a shared abstraction that requires understanding six other
files before you can touch one. When a simulation changes, only one file changes.
No edge cases bleed across boundaries. No ripple effects. You can delete any file
and nothing else breaks.

**C, with structures and functions only.**
The whole project is written in plain C — no classes, no inheritance, no
interfaces, no module system. After enough years of professional work I came to
believe that those abstractions, sold as aids to understanding, often hinder it:
every layer is another file to open, another name to memorise, and another
contract to hold in your head before reaching the actual computation. When the
goal is to *understand how something works*, plain `struct`s and free functions
are the floor of what a program needs to express; anything beyond that has to
earn its keep. The cost is a little duplication; the benefit is that any program
can be read top-to-bottom in a single sitting, and every change stays exactly
where you put it. Easy to read, easy to maintain, hard to lose track of.

**Copying is the intended usage.**
To run any simulation, copy the file, compile, and run:
```bash
gcc filename.c -lncurses -lm && ./a.out
```
That is the entire workflow. No build system, no CMake, no Makefile, no package
manager, no project configuration. A single file is a single program. A learner
can take any file, read it top to bottom, and understand the whole thing.

**Linux terminal only. No Windows, no GUI.**
Every simulation targets a POSIX terminal with ncurses. The constraint is the
medium: forcing a Navier-Stokes solver or a path tracer through a character grid
demands a much sharper understanding of the underlying math than reaching for a
graphics API would. The terminal is not a limitation — it is the whole point.

**Art for art's sake.**
None of these simulations solve a practical problem. A falling-sand automaton has
no business case. A Buddhabrot renderer does not ship a product. A mushroom-cloud
raymarcher is not on anyone's roadmap. That uselessness is precisely what makes
them worth building. The best way to understand an algorithm is to make it
beautiful with no deadline and no stakeholder.

**One physics model, one rendering model, applied uniformly.**
Every file uses the same fixed-timestep accumulator, the same pixel-space
coordinate model, and the same ncurses double-buffer sequence. A reader who
studies one file can read any other. The framework is not hidden — it is the
first thing documented in every source file.

---

## Demos

Every program lives in a topic folder; folders are summarised here, and full
per-program algorithm notes are in [DEMOS.md](DEMOS.md).

| Folder | Programs | Summary |
|--------|---------:|---------|
| `fluid/` | 17 | Stable fluids (Stam), lattice gas / Boltzmann, FDTD wave solvers, Gray-Scott reaction-diffusion, Lenia, FitzHugh-Nagumo excitable medium, SPH, falling sand, marching squares, vorticity-streamfunction, CFL stability explorer |
| `physics/` | 31 | Lorenz / N-body / cloth / Ising / Schrödinger; Schwarzschild black hole; quaternion gyroscope; PBD chains; rigid-body cubes & spheres; soft-body jelly; Barnes-Hut O(N log N) gravity; LBM, mass-spring lattice, membrane FDTD; CG and multigrid solver visualisers; RK1/2/4 comparison; spectrogram |
| `fractal_random/` | 14 | Mandelbrot / Julia / Buddhabrot / Newton; interactive Julia explorer; Barnsley IFS chaos game; DLA + dielectric breakdown; Lyapunov fractal; logistic bifurcation; Apollonian; L-systems; Lorenz strange attractor |
| `misc/` | 6 + 4 | Conway Life + variants, 1D Wolfram CA, Langton's ant, hex Life, general 2D outer-totalistic CA; sort visualiser, maze DFS+BFS+A\*, graph search, Drossel-Schwabl forest fire |
| `raster/` + `raymarcher/` | 13 | SDF raymarcher (Blinn-Phong, shadow rays, primitives, smooth-union gallery); Mandelbulb explorer + rasterizer; UV torus / cube / sphere / displacement raster; Bresenham wireframe; donut; volumetric `nuke_v1` mushroom cloud; `sun` solar SDF |
| `raytracing/` | 5 | Analytic ray-tracing of sphere / cube / torus / capsule + Cornell-box Monte Carlo path tracer (Lambertian BRDF, cosine-hemisphere sampling, Russian roulette) |
| `flocking/` | 9 | Reynolds boids, shepherd herding, crowd steering (6 behaviours), two-faction battle sim, swarm digit animator, ant-colony pheromone, Wa-Tor predator-prey, SIR epidemic, Physarum slime mould |
| `turtle/` | 1 | Dual turtle polygon animator |
| `grids/` | 14+4 + 7+4 + 7+4 + 12+24 = 76 | All grid families (rectangular, polar, hexagonal, triangular). Each family has a *displays* sub-folder showing the bare grid and a *placement* sub-folder where a cursor / patterns / paths / scatters deposit objects on it. Triangular family covers regular tilings (1–6), recursive fractals (7–9), aperiodic substitutions (10, 12), and Delaunay (11) |
| `geometry/` | 16 | Lissajous, spirograph, string-art; Voronoi, convex hull, Delaunay; k-d tree, BSP tree, quadtree; visibility polygon |
| `artistic/` | ≈30 | Epicycles + Fourier art, FFT visualiser, cymatics; plasma, aurora, Penrose pentagrid; diamond-square + Perlin terrain; Matrix-rain variants (rain / DLA-snowflake / pulsar / sun-mask); LED & particle digit morphing; spiral galaxy, jellyfish, gear, transit map, fireworks-rain; bonsai L-system, falling leaves, DNA helix, Dune sandworm/rocket |
| `animation/` + `robots/` | 14 | Hexapod tripod gait, IK spider / arm-reach / tentacle-seek, ragdoll figure & ropes, FK snake / centipede / tentacle-forest / medusa, walking biped, spring-leg jumper, self-balancing Perlin-terrain bot |
| `particle_systems/` | 8 | Fire (3 algos), smoke (3 algos), fireworks FSM, kaboom shockwave, generic particle sandbox, AAlib fire port, staggered burst, constellation network |
| `matrix_rain/` | 4 | Classic Matrix rain plus snowflake / pulsar / sun-mask hybrids |
| `ncurses_basics/` | — | Framework reference programs (`bounce_ball` lives in `physics/`) |

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

See `CLAUDE.md` for the complete build list.

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

## Documentation

`documentation/learning/` contains a `concept_*.md` file for every program in the project. Each file has two passes:

- **Pass 1** — core idea, mental model, key equations, data structures, non-obvious design decisions, open questions to explore
- **Pass 2** — pseudocode, module map, data flow diagram, core loop

`documentation/Master.md` is the cross-cutting reference: each technique used in the project is explained from first principles, names the source files where it appears, and (for named algorithms) cites the canonical paper, textbook, or author.

---

## Dependencies

- `gcc` (C11)
- `libncurses` (`sudo apt install libncurses5-dev` or equivalent)
- `libm` (standard, always available)

Nothing else. No CMake, no package manager, no runtime dependencies.
