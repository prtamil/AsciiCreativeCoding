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

### 7. `sandpile.c` — Bak-Tang-Wiesenfeld Sandpile
Drop grains at centre; cells with ≥ 4 grains topple to neighbours.
Avalanche cascades produce 1/f power-law size distribution — self-organised criticality.
Fractal patterns emerge from simple local rule.  Color by grain count (0→3).
**Challenge:** efficient avalanche propagation; clean reset when pile reaches screen edge.

### 8. `penrose.c` — Penrose Tiling (P3 Rhombus)
Recursive substitution inflation of thick + thin rhombus tiles.
Aperiodic — never repeats.  Slowly zoom in, revealing self-similarity at every scale.
Color by tile type and generation depth.
**Challenge:** substitution rule coded as edge-midpoint subdivision; zoom without integer rounding gaps.

### 9. `terrain.c` — Fractal Terrain (Diamond-Square)
Diamond-square midpoint displacement on a 2^n+1 grid.
Render as ASCII elevation contours: `·` lowland → `-` foothills → `^` peaks → `*` snowcap.
Slowly erode the heightmap each tick (thermal weathering) so mountains crumble to plains.
**Challenge:** wrapping the algorithm for seamless tiling; mapping float heights to discrete contour bands without ugly banding.

---

## artistic/

### 10. `lissajous.c` — Harmonograph / Lissajous
Two perpendicular damped oscillators; frequency ratio and phase drift slowly.
Trace the evolving parametric knot — figure-8s, stars, petals, spirals in sequence.
Auto-cycle through rational frequency ratios (1:1, 2:3, 3:4, 3:5 …).
**Challenge:** slow enough phase drift to reveal each shape fully before transitioning.

### 11. `aurora.c` — Aurora Borealis
Layered fBm noise bands scrolling horizontally, 3–5 vertical curtain columns.
Green/cyan core, purple/pink fringes, brightness shimmer from a second noise octave.
Star field background (static dots).  Pure colour-art — no physics.
**Challenge:** multi-layer noise blending in 256-color pairs without banding artefacts.

### 12. `voronoi.c` — Animated Voronoi
20–30 seed points moving with slow Brownian drift.
Each cell coloured by seed index; cell boundaries drawn with `·` dots.
Seeds "bounce" off edges.  Optional: Fortune's algorithm HUD showing sweep line.
**Challenge:** per-frame brute-force nearest-seed search fast enough at terminal resolution.

### 13. `spirograph.c` — Spirograph (Hypotrochoid / Epitrochoid)
Parametric r, R, d with slow parameter drift.  Multiple simultaneous curves
in different colors, slightly out of phase.  Fade old curves so new ones emerge.
**Challenge:** smooth Bresenham rasterization of the dense parametric curve without gaps.

---

## misc/

### 14. `sort_vis.c` — Sorting Algorithm Visualiser
Animated vertical bar chart.  Algorithms: bubble, insertion, selection, merge,
quicksort, heapsort, radix.  Color: unsorted grey → comparison cyan → swap red → sorted green.
Key-cycle algorithms; speed control.  Step counter and comparison count HUD.
**Challenge:** coroutine-style stepper so each algorithm yields one comparison/swap per tick.

---

## raymarcher/

### 15. `metaballs.c` — SDF Metaballs + Blending
5–8 metaballs moving on Lissajous paths; smooth-min SDF blending (k-factor).
Phong shading + soft shadows.  Vary k live to morph between separate spheres
and fully merged blob.  Color by surface curvature.
**Challenge:** smooth-min derivative for correct normals; soft-shadow ray offset tuning.

---

## Suggested build order

| Priority | File | Reason |
|---|---|---|
| 1 | `lorenz.c`     | Natural companion to double_pendulum; iconic shape |
| 2 | `nbody.c`      | Extends double_pendulum physics folder nicely |
| 3 | `terrain.c`    | Diamond-square is short; striking contour output |
| 4 | `aurora.c`     | Pure artistic; no physics, just colour layering |
| 5 | `sandpile.c`   | Short, surprising, fits fractal_random folder |
| 6 | `lissajous.c`  | Natural companion to epicycles and string_art |
| 7 | `spirograph.c` | Similar to lissajous, easy after it |
| 8 | `sort_vis.c`   | Educational; visually satisfying |
| 9 | `cloth.c`      | Most complex physics; save for after simpler ones |
| 10 | `metaballs.c` | Extends raymarcher folder naturally |
