# Next 20 Programs

Suggested additions to the collection, grouped by folder.
Each entry notes the core algorithm, what makes it visually distinct, and the key technical challenge.

---

## physics/

### 1. `lorenz.c` — Lorenz Strange Attractor
Integrate the three Lorenz ODEs (σ=10, ρ=28, β=8/3) with RK4.
Project the 3-D trajectory onto 2-D with a slow auto-rotating view angle.
Ring-buffer trail fades red→orange→grey.  Second ghost trajectory (ε offset)
diverges on the Lyapunov time-scale — companion piece to double_pendulum chaos.
**Challenge:** aspect-correct 3-D→2-D projection; rotation without distorting the attractor shape.

### 2. `nbody.c` — N-Body Gravity
15–30 point masses with Verlet integration, softened 1/r² gravity.
Trails reveal orbital paths, slingshot ejections, figure-8 solutions.
Optional: fixed central black-hole mass.
**Challenge:** O(n²) force loop fast enough at 30 fps; softening ε prevents singularities.

### 3. `cloth.c` — Spring-Mass Cloth
Grid of masses connected by structural + shear + bend springs.
Top row pinned; gravity + wind perturbation.  Verlet + constraint relaxation.
Fold lines, billowing, and corner tucks emerge naturally.
**Challenge:** position-based constraint solver (Jakobsen) stable at large timesteps.

### 4. `gyroscope.c` — Spinning Top / Euler Equations
Rigid body torque-free precession via Euler's rotation equations.
Draw the body frame axes rotating around the angular momentum vector.
Optionally add gravity-driven nutation — the wobble tightens as spin increases.
**Challenge:** integrating SO(3) rotation matrix without drift; Gram-Schmidt re-orthogonalisation.

---

## fluid/

### 5. `navier_stokes.c` — Stable Fluid (Jos Stam)
Velocity + density grid, advect-project scheme.  Inject dye with key presses.
Vortices, swirls, and diffusion emerge.  Arrow-key "wind" source.
**Challenge:** pressure-projection Poisson solve via Gauss-Seidel; advection without artificial dissipation.

### 6. `lenia.c` — Lenia (Continuous Life)
Continuous-state, continuous-time generalisation of Game of Life.
Convolution kernel + growth function; smooth, organic "creatures" move and divide.
**Challenge:** efficient 2-D convolution at terminal resolution; kernel/growth parameter UI.

---

## fractal_random/

### 7. `penrose.c` — Penrose Tiling (P3 Rhombus)
Recursive substitution inflation of thick + thin rhombus tiles.
Aperiodic — never repeats.  Slowly zoom in, revealing self-similarity at every scale.
Color by tile type and generation depth.
**Challenge:** substitution rule coded as edge-midpoint subdivision; zoom without integer rounding gaps.

### 8. `terrain.c` — Fractal Terrain (Diamond-Square)
Diamond-square midpoint displacement on a 2^n+1 grid.
Render as ASCII elevation contours: `·` lowland → `-` foothills → `^` peaks → `*` snowcap.
Slowly erode the heightmap each tick (thermal weathering) so mountains crumble to plains.
**Challenge:** wrapping the algorithm for seamless tiling; mapping float heights to discrete contour bands without ugly banding.

---

## artistic/

### 9. `lissajous.c` — Harmonograph / Lissajous
Two perpendicular damped oscillators; frequency ratio and phase drift slowly.
Trace the evolving parametric knot — figure-8s, stars, petals, spirals in sequence.
Auto-cycle through rational frequency ratios (1:1, 2:3, 3:4, 3:5 …).
**Challenge:** slow enough phase drift to reveal each shape fully before transitioning.

### 10. `aurora.c` — Aurora Borealis
Layered fBm noise bands scrolling horizontally, 3–5 vertical curtain columns.
Green/cyan core, purple/pink fringes, brightness shimmer from a second noise octave.
Star field background (static dots).  Pure colour-art — no physics.
**Challenge:** multi-layer noise blending in 256-color pairs without banding artefacts.

### 11. `voronoi.c` — Animated Voronoi
20–30 seed points moving with slow Brownian drift.
Each cell coloured by seed index; cell boundaries drawn with `·` dots.
Seeds "bounce" off edges.  Optional: Fortune's algorithm HUD showing sweep line.
**Challenge:** per-frame brute-force nearest-seed search fast enough at terminal resolution.

### 12. `spirograph.c` — Spirograph (Hypotrochoid / Epitrochoid)
Parametric r, R, d with slow parameter drift.  Multiple simultaneous curves
in different colors, slightly out of phase.  Fade old curves so new ones emerge.
**Challenge:** smooth Bresenham rasterization of the dense parametric curve without gaps.

### 13. `plasma.c` — Plasma / Colour Wave
Classic demoscene plasma: sum of sinusoids evaluated at each terminal cell and mapped
through a cycling 256-colour palette.  No physics — just `sin(x·f1 + t) + sin(y·f2 + t) + …`
summed and palette-indexed.  Frequency knobs and palette themes cycle with keys.
Visually hypnotic; trivially cheap to compute.
**Challenge:** smooth cycling palette without banding; multiple frequency combos that look
distinct rather than all washing into the same blob.

---

## misc/

### 14. `sort_vis.c` — Sorting Algorithm Visualiser
Animated vertical bar chart.  Algorithms: bubble, insertion, selection, merge,
quicksort, heapsort, radix.  Color: unsorted grey → comparison cyan → swap red → sorted green.
Key-cycle algorithms; speed control.  Step counter and comparison count HUD.
**Challenge:** coroutine-style stepper so each algorithm yields one comparison/swap per tick.

### 15. `maze.c` — Maze Generation + Solve
Recursive-backtracker DFS generates the maze wall by wall — watch the frontier advance.
Once complete, BFS/A* animates the solution path in a contrasting colour.
Presets: small fast maze vs large slow maze.  Reset re-generates with new random seed.
**Challenge:** encoding walls as edge bits (4 bits per cell) to avoid a separate wall grid;
smooth cell-by-cell draw during generation without full redraw each frame.

### 16. `ising.c` — Ising Model (Magnetic Phase Transition)
Each cell is a spin: `+1` (up) or `−1` (down).  Monte Carlo Metropolis algorithm flips
spins according to the Boltzmann factor `exp(−ΔE / kT)`.  At high T spins are random
noise; cool below the critical temperature T_c and magnetic domains spontaneously form —
regions of aligned spins growing to fill the screen.  HUD shows temperature and mean
magnetisation.  Keys raise/lower T so the phase transition is observable live.
**Challenge:** efficient neighbour-energy calculation; finding T_c for the terminal aspect ratio.

---

## fractal_random/ (additional)

### 17. `l_system.c` — L-System Fractal Plants
String-rewriting L-system with turtle-graphics interpretation.
Five presets: Dragon Curve, Hilbert Curve, Sierpinski Arrow, Branching Plant, Koch Island.
Each preset shows one iteration at a time, building the fractal generation by generation.
Color by recursion depth.
**Challenge:** variable-length string budget (exponential growth); fitting the turtle path
within terminal bounds by auto-scaling the step length per generation.

### 18. `automaton_2d.c` — Larger-than-Life / Extended CA
Generalised 2-D cellular automaton: configurable neighbourhood radius R (1–5),
count thresholds, and state count.  Radius-2 rules produce exotic crystal structures,
spirals, and moving "blobs" impossible in standard GoL.  Presets: Bosco's rule,
Larger-than-Life Vote, Snowflakes, Moving Bands.
Color by cell state (0..N-1) using 256-colour palette.
**Challenge:** efficient summing of R×R neighbourhood for large R without O(R²) per cell;
toroidal edge handling.

---

## Suggested build order

| Priority | File | Reason |
|---|---|---|
| 1  | `lorenz.c`        | Natural companion to double_pendulum; iconic attractor shape |
| 2  | `nbody.c`         | Extends physics folder; Verlet gravity well-understood |
| 3  | `terrain.c`       | Diamond-square is short; striking contour output |
| 4  | `aurora.c`        | Pure artistic; no physics, just colour layering |
| 5  | `plasma.c`        | Simplest possible; pure sin-sum palette — good warmup |
| 6  | `lissajous.c`     | Natural companion to epicycles and string_art |
| 7  | `spirograph.c`    | Similar to lissajous, easy after it |
| 8  | `sort_vis.c`      | Educational; visually satisfying |
| 9  | `maze.c`          | Wall-bit encoding + BFS solve; self-contained |
| 10 | `ising.c`         | Monte Carlo Metropolis; striking phase transition |
| 11 | `voronoi.c`       | Brute-force nearest-seed; tractable at terminal resolution |
| 12 | `penrose.c`       | Aperiodic tiling; recursive substitution |
| 13 | `l_system.c`      | String rewriting + turtle graphics; builds on fractal folder |
| 14 | `lenia.c`         | Continuous Game of Life; builds on life.c knowledge |
| 15 | `navier_stokes.c` | Stable fluid; most complex fluid sim; save for after wave.c experience |
| 16 | `gyroscope.c`     | Euler rotation equations; builds on double_pendulum RK4 |
| 17 | `cloth.c`         | Most complex physics; Jakobsen constraint solver |
| 18 | `automaton_2d.c`  | Extends life.c; larger neighbourhood = new pattern classes |
