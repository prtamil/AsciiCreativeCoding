# Next 20 Programs

Suggested additions to the collection, grouped by folder.
Each entry notes the core algorithm, what makes it visually distinct, and the key technical challenge.

---

## physics/

### 1. `pendulum_wave.c` — Pendulum Wave
N=15 pendulums with lengths chosen so periods are harmonically related: `L_n = L / (1 + n·k)`.
At t=0 all swing in phase; they gradually drift into beautiful wave patterns, then "clap" back to sync.
Each pendulum is a colored vertical line; collective patterns emerge from pure phase arithmetic.
**Challenge:** choosing the period ratios so the resync time is ~60 s and intermediate patterns are striking.

### 2. `elastic_collision.c` — Hard Sphere Billiards
20–40 discs with randomised radii and velocities bouncing off walls and each other.
Perfectly elastic collisions (kinetic energy + momentum conserved); color shifts on impact to highlight the transfer.
Shows Maxwell–Boltzmann velocity distribution emerging spontaneously.
**Challenge:** broad-phase spatial hashing to avoid O(n²) pair checks at every tick.

---

## fluid/

### 3. `navier_stokes.c` — Stable Fluid (Jos Stam)
Velocity + density grid, advect-project scheme.  Inject dye with key presses.
Vortices, swirls, and diffusion emerge.  Arrow-key "wind" source.
**Challenge:** pressure-projection Poisson solve via Gauss-Seidel; advection without artificial dissipation.

### 4. `sph.c` — Smoothed Particle Hydrodynamics
80–120 particles as fluid parcels; pressure and viscosity computed from kernel-weighted neighbours.
Particles pile up, slosh, and form droplets under gravity.  Mouse-key impulse injects momentum.
**Challenge:** SPH kernel (cubic spline), neighbour search via cell grid, tuning rest density to prevent jitter.

### 5. `lenia.c` — Lenia (Continuous Life)
Continuous-state, continuous-time generalisation of Game of Life.
Convolution kernel + growth function; smooth, organic "creatures" move and divide.
**Challenge:** efficient 2-D convolution at terminal resolution; kernel/growth parameter UI.

---

## fractal_random/

### 6. `bifurcation.c` — Logistic Map Bifurcation Diagram
Sweep r from 2.4 → 4.0; for each r run 500 warmup iterations then plot the next 300 x-values.
The classic period-doubling cascade and Feigenbaum universality emerge as a branching tree.
Scroll left/right to zoom into the fractal structure at the edge of chaos.
**Challenge:** fitting 1600 r-values across terminal columns; suppressing transient iterations cleanly.

### 7. `newton_fractal.c` — Newton's Method Fractal
Apply Newton's root-finding iteration to z^4 − 1 = 0 in the complex plane.
Basin boundaries form a fractal — zoom reveals infinite self-similar cusps.
Color by which root the iteration converged to; brightness by convergence speed.
**Challenge:** smooth coloring between basins; zoom without banding at the boundary.

### 8. `burning_ship.c` — Burning Ship Fractal
Same escape-time iteration as Mandelbrot but with `z → (|Re(z)| + i|Im(z)|)² + c`.
The absolute value folds the plane, creating a ship-like hull structure and flame-shaped filaments.
Color with a fire palette; slow zoom into the ship's "deck" region.
**Challenge:** the asymmetry (no left-right symmetry) means the interesting region must be found by panning.

### 9. `strange_attractor.c` — Point Density Attractor
Clifford / de Jong / Ikeda attractor: iterate a 2-D map millions of times, accumulate hit counts in a density grid.
Log-normalize and map density to a nebula palette (black → blue → white).
Cycle through 6 named attractors with key presses; smooth crossfade between parameter sets.
**Challenge:** density accumulation in a fixed integer grid; log normalization without clipping bright cores.

---

## artistic/

### 10. `ant_colony.c` — Pheromone Stigmergy
50 ants wander, lay pheromone trails, and follow gradients toward food sources.
Pheromone evaporates over time; shortest paths emerge from collective reinforcement.
Two food sources visible simultaneously; watch separate trails compete then merge.
**Challenge:** pheromone grid deposit/evaporate without per-cell allocation; bias random walk toward gradient.

### 11. `fourier_draw.c` — Fourier Epicycle Reconstruction
Precomputed DFT of a parametric closed path (star, trefoil, batman, figure-8…).
Draw the full epicycle arm chain rotating in real time, trailing the reconstructed curve.
Cycle the number of arms from 1 → N to show convergence live.
**Challenge:** matching epicycles.c architecture but with path-specific DFT coefficients rather than shape IFS.

### 12. `reaction_wave.c` — FitzHugh-Nagumo Excitable Media
Two-variable PDE on the terminal grid: activator u and inhibitor v.
Trigger a point impulse → expanding ring wave. Wrong parameters → spiral wave rotating forever.
4 presets: target rings, spiral pair, chaos, plane wave.
**Challenge:** FitzHugh-Nagumo PDE stability (explicit Euler needs dt < 0.1); stacking with Gray-Scott color theme code.

---

## misc/

### 13. `sort_vis.c` — Sorting Algorithm Visualiser
Animated vertical bar chart.  Algorithms: bubble, insertion, selection, merge,
quicksort, heapsort, radix.  Color: unsorted grey → comparison cyan → swap red → sorted green.
Key-cycle algorithms; speed control.  Step counter and comparison count HUD.
**Challenge:** coroutine-style stepper so each algorithm yields one comparison/swap per tick.

### 14. `maze.c` — Maze Generation + Solve
Recursive-backtracker DFS generates the maze wall by wall — watch the frontier advance.
Once complete, BFS/A* animates the solution path in a contrasting colour.
Presets: small fast maze vs large slow maze.  Reset re-generates with new random seed.
**Challenge:** encoding walls as edge bits (4 bits per cell) to avoid a separate wall grid;
smooth cell-by-cell draw during generation without full redraw each frame.

### 15. `ising.c` — Ising Model (Magnetic Phase Transition)
Each cell is a spin: `+1` (up) or `−1` (down).  Monte Carlo Metropolis flips spins
according to `exp(−ΔE / kT)`.  At high T spins are random noise; cool below T_c and
magnetic domains spontaneously form.  HUD shows temperature and mean magnetisation.
**Challenge:** efficient neighbour-energy calculation; finding T_c for the terminal aspect ratio.

### 16. `schrodinger.c` — 1-D Quantum Wavefunction
Split-operator FFT method: evolve ψ under H = −ℏ²/2m · ∂²/∂x² + V(x).
Gaussian wave-packet tunnels through a finite barrier; reflection/transmission coefficients shown live.
Color real part blue, imaginary part red, |ψ|² white; probability density fills the screen.
**Challenge:** split-operator requires FFT forward/inverse each tick; normalization drift over long runs.

### 17. `perlin_landscape.c` — Perlin Terrain Flythrough
Raycast a 2-D Perlin heightmap column by column: for each screen column cast a ray and find the
max-angle intersection, drawing sky above and terrain below with depth-based fog.
Camera drifts forward automatically; `←→` steer, `↑↓` altitude.
**Challenge:** ray–heightmap intersection without a full SDF; horizon-only draw per column avoids overdraw.

### 18. `graph_search.c` — Animated Pathfinding
Random planar graph of 60–80 nodes with visible edges.  Run BFS, DFS, or A* live:
frontier nodes pulse cyan, visited grey, shortest path red.  Key switches between algorithms.
Side-by-side mode shows BFS and A* expanding simultaneously for direct comparison.
**Challenge:** force-directed layout to space nodes legibly; priority queue for A* without dynamic alloc.

### 19. `network_sim.c` — SIR Epidemic on a Small-World Network
100 nodes on a Watts–Strogatz ring with p=0.1 rewiring; nodes are S (white), I (red), R (green).
One infected seed spreads via edge contacts with probability β; recovered nodes are immune.
R0 = β/γ HUD; `↑↓` tune β live and watch epidemic fade vs explode.
**Challenge:** Watts–Strogatz rewiring at init; per-edge stochastic contact without O(E) per tick.

### 20. `ca_music.c` — Musical Cellular Automaton
Rule-110 or Langton's Ant drives a 1-D pitch row: live cells select a scale note, dead cells rest.
ANSI escape codes trigger the terminal bell on beat — produces generative rhythmic music.
Visualise the automaton grid while audio plays; `t` changes tempo, `s` changes scale.
**Challenge:** mapping CA state to musical rhythm without audio library; timer alignment for beat-accurate bell.

---

## Suggested build order

| Priority | File | Reason |
|---|---|---|
| 1  | `bifurcation.c`    | Short — 1-D map; direct companion to lorenz.c chaos |
| 2  | `sort_vis.c`       | Educational; visually satisfying; straightforward |
| 3  | `maze.c`           | Wall-bit encoding + BFS solve; self-contained |
| 4  | `pendulum_wave.c`  | Natural companion to double_pendulum; phase math only |
| 5  | `burning_ship.c`   | Trivial extension of mandelbrot.c |
| 6  | `newton_fractal.c` | Escape-time with complex Newton; builds on julia.c |
| 7  | `strange_attractor.c` | Density accumulator; builds on buddhabrot.c pattern |
| 8  | `ising.c`          | Monte Carlo Metropolis; striking phase transition |
| 9  | `ant_colony.c`     | Pheromone stigmergy; builds on langton.c knowledge |
| 10 | `fourier_draw.c`   | Extends epicycles.c with path-specific DFT |
| 11 | `elastic_collision.c` | Spatial hashing + collision response |
| 12 | `reaction_wave.c`  | FitzHugh-Nagumo; builds on reaction_diffusion.c |
| 13 | `graph_search.c`   | Force-directed layout + BFS/A* animation |
| 14 | `sph.c`            | Particle-based fluid; complements navier_stokes |
| 15 | `lenia.c`          | Continuous Game of Life; builds on life.c knowledge |
| 16 | `navier_stokes.c`  | Stable fluid; most complex fluid sim |
| 17 | `perlin_landscape.c` | Raycasting heightmap; new rendering technique |
| 18 | `network_sim.c`    | SIR epidemic; small-world graph construction |
| 19 | `schrodinger.c`    | FFT split-operator; requires complex arithmetic |
| 20 | `ca_music.c`       | Terminal bell timing; most experimental |
