# Terminal Demos — ncurses C Projects

## Build

```bash
# ── basics ───────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/tst_lines_cols.c  -o tst_lines_cols  -lncurses
gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/aspect_ratio.c    -o aspect_ratio    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/spring_pendulum.c -o spring_pendulum -lncurses -lm

# ── misc ─────────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra misc/sort_vis.c          -o sort_vis        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra misc/maze.c              -o maze            -lncurses
gcc -std=c11 -O2 -Wall -Wextra misc/ca_music.c          -o ca_music        -lncurses -lm

# ── matrix ───────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra matrix_rain/matrix_rain.c     -o matrix_rain     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra matrix_rain/pulsar_rain.c     -o pulsar_rain     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra matrix_rain/sun_rain.c        -o sun_rain        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra matrix_rain/fireworks_rain.c  -o fireworks_rain  -lncurses -lm

# ── particle systems ─────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra particle_systems/fire.c         -o fire         -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/aafire_port.c  -o aafire        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/fireworks.c    -o fireworks     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/brust.c        -o brust         -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/kaboom.c       -o kaboom        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/constellation.c -o constellation -lncurses -lm

# ── flocking ─────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra flocking/flocking.c             -o flocking      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra flocking/shepherd.c             -o shepherd      -lncurses -lm

# ── fluid / grid sims ────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra fluid/sand.c                    -o sand                -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/flowfield.c               -o flowfield           -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/reaction_diffusion.c      -o reaction_diffusion  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/wave.c                    -o wave                -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/wave_2d.c                 -o wave_2d             -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/reaction_wave.c           -o reaction_wave       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/navier_stokes.c           -o navier_stokes       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/lenia.c                   -o lenia               -lncurses -lm

# ── fractal / random growth ───────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra fractal_random/snowflake.c  -o snowflake  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/coral.c      -o coral      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/sierpinski.c -o sierpinski -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/fern.c       -o fern       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/julia.c      -o julia      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/mandelbrot.c -o mandelbrot -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/koch.c       -o koch       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/lightning.c    -o lightning    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/buddhabrot.c  -o buddhabrot   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/sandpile.c    -o sandpile     -lncurses

# ── physics ──────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra physics/bounce_ball.c        -o bounce            -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/double_pendulum.c    -o double_pendulum   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/lorenz.c             -o lorenz            -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/nbody.c              -o nbody             -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/cloth.c              -o cloth             -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/pendulum_wave.c      -o pendulum_wave     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/elastic_collision.c  -o elastic_collision -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/orbit_3body.c        -o orbit_3body       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/ising.c              -o ising             -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/schrodinger.c        -o schrodinger       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/blackhole.c          -o blackhole         -lncurses -lm

# ── fractal / random growth (new) ────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra fractal_random/penrose.c           -o penrose           -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/terrain.c           -o terrain           -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/perlin_landscape.c  -o perlin_landscape  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/bifurcation.c    -o bifurcation      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/burning_ship.c   -o burning_ship     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/newton_fractal.c -o newton_fractal   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/strange_attractor.c -o strange_attractor -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/dragon_curve.c   -o dragon_curve     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/apollonian.c     -o apollonian       -lncurses -lm

# ── artistic ──────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra artistic/bonsai.c            -o bonsai       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra Artistic/bat.c               -o bat          -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra Artistic/2stroke.c           -o 2stroke      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/leaf_fall.c         -o leaf_fall    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/epicycles.c         -o epicycles    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/cellular_automata_1d.c -o cellular_automata_1d -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/life.c                -o life                -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/langton.c             -o langton             -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/cymatics.c            -o cymatics            -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/wator.c               -o wator               -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/aurora.c              -o aurora              -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/plasma.c              -o plasma              -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/fourier_draw.c        -o fourier_draw  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/ant_colony.c          -o ant_colony    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/graph_search.c        -o graph_search  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/network_sim.c         -o network_sim   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/hex_life.c            -o hex_life      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/jellyfish.c           -o jellyfish     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/xrayswarm.c           -o xrayswarm     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/gear.c                -o gear          -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/railwaymap.c          -o railwaymap    -lncurses -lm

# ── raytracing ───────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra raytracing/sphere_raytrace.c   -o sphere_raytrace   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raytracing/cube_raytrace.c     -o cube_raytrace     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raytracing/torus_raytrace.c    -o torus_raytrace    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raytracing/capsule_raytrace.c  -o capsule_raytrace  -lncurses -lm

# ── geometry ──────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra geometry/rect_grid.c    -o rect_grid   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra geometry/polar_grid.c   -o polar_grid  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra geometry/hex_grid.c     -o hex_grid    -lncurses
gcc -std=c11 -O2 -Wall -Wextra geometry/lissajous.c    -o lissajous   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra geometry/spirograph.c   -o spirograph  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra geometry/string_art.c   -o string_art  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra geometry/voronoi.c      -o voronoi     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra geometry/convex_hull.c  -o convex_hull -lncurses -lm

# ── raster (software rasterizer) ─────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra raster/torus_raster.c    -o torus    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raster/cube_raster.c     -o cube     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raster/sphere_raster.c   -o sphere   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raster/displace_raster.c -o displace -lncurses -lm

# ── raymarcher / 3-D ─────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra raymarcher/donut.c                -o donut       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raymarcher/wireframe.c            -o wireframe   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher.c           -o raymarcher  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher_cube.c      -o ray_cube    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raymarcher/raymarcher_primitives.c -o ray_prims  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raymarcher/metaballs.c            -o metaballs  -lncurses -lm
```

---

## Files

### ncurses_basics/
- `tst_lines_cols.c`    — print terminal `LINES` × `COLS` using `printw` / `refresh`
- `aspect_ratio.c`      — draw a correct-looking circle using `newwin` + 2× x-scaling
- `spring_pendulum.c`   — Lagrangian spring-pendulum with Bresenham spring coil and render lerp

### misc/
- `sort_vis.c`  — sorting visualiser: 5 algorithms (bubble/insertion/selection/quicksort/heapsort) as animated vertical bar chart; one operation per tick; grey/cyan/red/green color states; comparison + swap counters
- `maze.c`      — recursive-backtracker DFS maze generation; BFS/A* animated solve path; wall-bit encoding (4 bits per cell); small/large presets
- `ca_music.c`  — musical cellular automaton: 8 CA rules (110/30/90/150…); beat cursor sweeps width; live cells ring terminal bell (`\a`) on beat; 5 scales; tempo control; 7-color rainbow display

### matrix_rain/
- `matrix_rain.c`       — Matrix-style falling character rain: two-pass draw, theme system, render interpolation for smooth column-head scrolling
- `pulsar_rain.c`       — Rotating pulsar neutron-star: N evenly-spaced beams (1–16) sweep continuously; each leaves a 16-slot angular wake of fading matrix chars across 80 radial samples; '@' core always drawn last; +/- spin, [ ] beam count, t theme, r reset
- `sun_rain.c`          — Matrix Rain Sun: single '@' core at centre, 180 independent radial solar-wind streams (2° per ray) shoot outward continuously; random stagger offsets give stochastic beam field; ri < 1.0 clip preserves centre char; 5 themes; t r
- `fireworks_rain.c`   — Fireworks with matrix-rain arc trails: rockets explode at apex, each of 72 sparks grows a 16-slot position-history trail; characters shimmer 75 % per tick; head=white/bold, trail fades by color pair; 5 themes (vivid/matrix/fire/ice/plasma) remap all 7 spark color pairs; ] [ = - t

### particle_systems/
- `fire.c`              — Doom-style fire CA: heat diffusion, Floyd-Steinberg dithering, 6 auto-cycling color themes, wind and gravity controls
- `aafire_port.c`       — aalib fire variant: 5-neighbour CA, per-row decay LUT, 9-step `attr_t` brightness gradient
- `fireworks.c`         — rocket fireworks: IDLE → RISING → EXPLODED state machine, particle pool, gravity + drag
- `brust.c`             — random explosion bursts: staggered particle waves, scorch mark persistence, `A_DIM` residue rendering
- `kaboom.c`            — deterministic LCG explosions: same seed → same explosion shape, color-ring blast zones
- `constellation.c`     — star constellation: Bresenham stippled lines with `cell_used[][]` dedup, proximity `A_BOLD`, `prev/cur` lerp interpolation

### flocking/
- `flocking.c`          — boid flocking: 3 flock groups, 5 switchable modes (classic boids, leader chase, Vicsek, orbit, predator-prey), toroidal wrap, cosine palette color cycling, `A_BOLD` proximity halo
- `shepherd.c`          — shepherd herding sim: user-controlled `#` shepherd moves with arrow keys; sheep flock with boids (separation+alignment+cohesion) and flee when shepherd enters FLEE_RADIUS; sheep chars `o`(calm) `<>^v/\`(moving) `O`(fleeing); bounced boundaries so sheep can be cornered; dotted flee-radius ring toggle (`f`)

### fluid/
- `sand.c`                  — falling sand CA
- `reaction_diffusion.c`   — Gray-Scott model: 7 presets (Mitosis/Coral/Stripes/Worms/Maze/Bubbles/Solitons), 9-point isotropic Laplacian, 4 colour themes (ocean/forest/magma/violet), 600-step warmup pre-run, auto-cycle theme
- `wave.c`                 — FDTD 2-D wave equation: 5 oscillating sources (keys 1–5 toggle), Gaussian impulse (p), interference fringes + boundary reflections, 9-level signed amplitude display, 4 colour themes (water/lava/plasma/matrix), CFL-stable c=0.45
- `wave_2d.c`              — 2-D scalar wave equation (∂²u/∂t²=c²∇²u): point sources emit circular wavefronts, multiple sources interfere; CFL-stable explicit Euler; signed amplitude colour map (blue/black/white)
- `reaction_wave.c`        — FitzHugh-Nagumo excitable medium: two-variable activator/inhibitor PDE on terminal grid; spiral waves, target waves; 4 colour themes; interactive stimulus injection
- `flowfield.c`         — Perlin noise flow field: 3-octave fBm, bilinear field sampling, 8-direction arrow glyphs, ring-buffer particle trails
- `navier_stokes.c`     — Jos Stam stable fluid: velocity+density N×N grid, Gauss-Seidel diffuse (warm-start), semi-Lagrangian advect, divergence-free project; two counter-rotating auto-emitters; dynamic density normalization; 3 dye colors; arrow-key wind; pre-warmed 80 steps at startup
- `lenia.c`             — continuous Game of Life: convolution kernel + growth function, smooth organic creatures, multiple presets

### physics/
- `bounce_ball.c`       — **reference implementation** — bouncing balls with pixel-space physics, fixed-timestep accumulator, render interpolation (alpha), SIGWINCH resize
- `double_pendulum.c`   — double pendulum with RK4 integration demonstrating chaos: ghost pendulum (θ₁+ε), 500-slot trail ring buffer, divergence HUD metric
- `lorenz.c`            — Lorenz strange attractor: RK4 integration, rotating orthographic 3-D→2-D projection, 1500-slot ring-buffer trail (red→grey), ghost trajectory showing Lyapunov chaos divergence
- `nbody.c`             — N-body gravity: 20 point masses, softened 1/r² (ε=4 px), Velocity Verlet, per-body 200-slot color-faded trails, optional central black hole
- `cloth.c`             — Spring-mass cloth: explicit spring forces (Hooke + velocity damping), symplectic Euler, 3 modes (hanging/flag/hammock), render lerp for smooth sub-tick motion
- `pendulum_wave.c`     — N=15 pendulums with harmonically-related lengths; ω_n=2π(N_base+n)/T_sync; ropes drawn as diagonal lines (`|` `/` `\`) from pivot to displaced bob; 15-step rainbow palette; amplitude and resync controls
- `elastic_collision.c` — hard-sphere billiards: 20–40 discs, perfectly elastic collisions (momentum+KE conserved), spatial-hash broad phase, Maxwell–Boltzmann velocity distribution emerges; color flashes on impact
- `orbit_3body.c`       — three-body gravity: Velocity Verlet, figure-8 stable orbit initial conditions, perturbation key → chaos; speed-colored fading trails (blue slow → white fast)
- `ising.c`             — 2-D Ising model: Monte Carlo Metropolis spin flips; `exp(−ΔE/kT)` acceptance; phase transition at T_c; magnetisation HUD; interactive temperature control
- `schrodinger.c`       — 1-D Schrödinger equation: Crank-Nicolson tridiagonal solver (Thomas algorithm); Gaussian wave-packet; 4 presets (free/barrier/harmonic/double-slit); absorbing walls; |ψ|² display
- `blackhole.c`         — Gargantua black hole GR raytracer: Schwarzschild null geodesics (RK4), precomputed lensing table, photon ring from min_r tracking, primary + secondary disk images, relativistic Doppler beaming D=[(1+β)/(1−β)]^1.5, gravitational redshift; 11 themes; +/- zoom

### fractal_random/
- `penrose.c`      — Penrose P3 rhombus tiling: de Bruijn pentagrid duality, O(1) per cell, parity-based thick/thin distinction, pentagrid edge detection for visible tile outlines (|/\-), slow rotation, 256-color warm/cool palette
- `terrain.c`           — Diamond-square fractal terrain: 65×65 heightmap, ROUGHNESS=0.60, thermal weathering erosion (TALUS=0.022), bilinear interpolation to any terminal size, 7 ASCII contour levels (~.−^#*)
- `perlin_landscape.c`  — 2-D parallax scrolling landscape: 3 terrain layers (far/mid/near) at speeds 0.12×/0.38×/1.0×; 5-octave fBm Perlin noise per layer; painter's algorithm; deterministic stars; arrow-key speed/direction
- `snowflake.c`    — DLA crystal with D6 6-fold symmetry; 12-way simultaneous freeze; distance-based 6-color ice palette (light-blue core → teal → white tips); context-sensitive chars (`*` `|` `-` `+` `/` `\`)
- `coral.c`        — anisotropic DLA: 8 bottom seeds, top-spawned walkers with downward bias; direction-dependent sticking probability; vivid coral/violet/yellow/lime/teal palette; auto-reset when tallest branch hits top quarter
- `sierpinski.c`   — Sierpinski triangle via chaos game (IFS): 3 vertices, random vertex → move halfway; N_PER_TICK=500, TOTAL_ITERS=50000; color by last chosen vertex (cyan/yellow/magenta); held then reset
- `fern.c`         — Barnsley Fern: 4-transform IFS (stem 1%, main 85%, left 7%, right 7%); N_PER_TICK=400, TOTAL_ITERS=80000; independent x/y scale to correct aspect; green gradient palette
- `julia.c`        — Julia set with Fisher-Yates random pixel reveal; 6 presets cycling (rabbit, spiral, dendrite, flame, seahorse, basilica); fire palette (white→yellow→orange→red); PIXELS_PER_TICK=60, MAX_ITER=128
- `mandelbrot.c`   — Mandelbrot set (z₀=0, z→z²+c); same Fisher-Yates fill as julia.c; 6 zoom presets including deep spirals; electric neon palette (magenta/purple/cyan/lime/yellow); MAX_ITER=256
- `koch.c`         — Koch snowflake: recursive midpoint subdivision; levels 1–5 cycle; Bresenham rasterization; adaptive segs_per_tick for ~2 s per level; 5-color vivid gradient (cyan→teal→lime→yellow→white)
- `lightning.c`    — fractal branching lightning: recursive tip branching (not DLA); tips grow downward with persistent lean bias, fork after MIN_FORK_STEPS; glow halo radius 2; color by depth (light-blue → teal → white); state machine ST_GROWING → ST_STRIKING → ST_FADING
- `buddhabrot.c`        — Buddhabrot density accumulator: two-pass orbit sampling (escape test then trace); 5 presets (buddha 500/2000, anti 100/500/1000); log-normalized density→color (mode-aware floor: 0.05 buddha / 0.25 anti); purple→white nebula palette
- `newton_fractal.c`    — Newton's method on z⁴−1=0; basin coloring by root (4 colors) + brightness by convergence speed; zoom presets
- `strange_attractor.c` — Clifford/de Jong/Ikeda point-density attractor; 6 named attractors; log-normalized hit-count grid; nebula palette (black→blue→white)
- `dragon_curve.c`      — paper-folding dragon curve L-system; iterative string rewrite; generation-depth rainbow coloring; per-turn animation
- `apollonian.c`        — Apollonian gasket: Descartes circle theorem recursive circle packing; depth-based fire palette; clips circles smaller than one cell

### Artistic/
- `bonsai.c`       — growing bonsai tree: recursive branch growth, 5 tree types, pot styles, message panel with ACS box-drawing chars, `use_default_colors`
- `bat.c`          — ASCII bat swarms in Pascal-triangle formation: 3 groups × (n_rows+1)(n_rows+2)/2 bats each; `+`/`-` resize rows 1–6 live; wing animation `/−\−`; 6 preset launch angles; staggered group launch; light-purple/cyan/pink groups
- `2stroke.c`      — 2-stroke engine cross-section: slider-crank kinematics (piston/rod/crankshaft), exhaust and transfer port open/close, spark at TDC, phase labels COMPRESSION/IGNITION/POWER/EXHAUST/SCAVENGING; `] [` RPM control
- `leaf_fall.c`    — ASCII tree with matrix-rain leaf fall
- `epicycles.c`    — Fourier epicycles: DFT of parametric shape (heart/star/trefoil/fig-8/butterfly), sorted-by-amplitude arm chain, auto-adds epicycles to show convergence; orbit circles, colour-faded trail
- `cellular_automata_1d.c` — Wolfram 1-D elementary CA: 17 preset rules (30/90/110/18/150…), builds top-down row-by-row, 5 Wolfram classes color-coded, auto-cycle, type any rule 0–255+Enter
- `life.c`    — Conway's Game of Life + 5 rule variants (HighLife/Day&Night/Seeds/Morley/2×2), 6-colour palette, Gosper gun/glider/acorn/R-pentomino seeds, 3-row scrolling population histogram
- `langton.c` — Langton's Ant + 7 multi-colour turmite rules (RL/LR/LLRR/RLR/LRRL…), 1–3 simultaneous ants, cell states colour-coded, ant shown as bold @
- `cymatics.c`  — Chladni figures: Z=cos(mπx)cos(nπy)−cos(nπx)cos(mπy), 20 (m,n) modes, nodal lines rendered with glow chars (@#*+.), smooth morph animation between modes, 4 colour themes
- `aurora.c`    — Aurora borealis: two-octave sinusoidal curtains with vertical sine envelope, color zones by row fraction (magenta fringe/cyan core/green base), deterministic star hash (no storage, no flicker)
- `plasma.c`    — Demoscene plasma: 4-component sin-sum (horizontal/vertical/diagonal/radial), palette cycling via time-offset, 4 frequency presets (gentle/energetic/grand/turbulent), 4 color themes, 14 PalEntry levels each
- `hex_life.c`  — Game of Life on hexagonal grid (offset-row layout, 6 neighbours); rule B2/S34; multiple rule presets; hex→terminal column mapping
- `jellyfish.c` — bioluminescent jellyfish: four-state physics pulse (IDLE sink → CONTRACT jet → GLIDE coast → EXPAND bloom); asymmetric bell deformation (width via `head_f`, height via `crown_f`); tentacle inertia lag + wave_scale gating; 8 color variants; +/− jellyfish count
- `xrayswarm.c` — multi-swarm pulsating X-ray rays: workers radiate from fixed queen (DIVERGE), park at screen edge (PAUSE), retrace exact path back to origin (CONVERGE); locked queen coords; 4-pass rendering (DIM/MID/BRIGHT/HEAD) across all swarms prevents trail cancellation
- `gear.c` — wireframe rotating gear with themed sparks: polar proximity edge detection (hub ring/spokes/tooth sides/arcs), tangential surface-velocity emission (tang_v = ω×R×TANG_SCALE), speed-proportional emission rate, 7-stage cooling gradient, 10 named 256-color themes (FIRE/MATRIX/PLASMA/NOVA/POISON/OCEAN/GOLD/NEON/ARCTIC/LAVA); t/T cycles themes live via init_pair re-binding
- `railwaymap.c` — procedural transit map: H/V/Z line templates on a 6×4 logical grid; stations emerge at every grid node; canvas-based ACS junction detection (h_cp+v_cp → ACS_PLUS); interchange nodes where ≥2 lines share a position; 10 color themes; r:new map

### misc/
- `sort_vis.c`  — sorting visualiser: 5 algorithms (bubble/insertion/selection/quicksort/heapsort) as animated vertical bar chart; one operation per tick; grey/cyan/red/green color states; comparison + swap counters
- `maze.c`      — recursive-backtracker DFS maze generation; BFS/A* animated solve path; wall-bit encoding (4 bits per cell); small/large presets
- `ca_music.c`  — musical cellular automaton: 8 CA rules (110/30/90/150…); beat cursor sweeps width; live cells ring terminal bell (`\a`) on beat; 5 scales; tempo control; 7-color rainbow display

### geometry/
- `rect_grid.c`   — rectangular character grid: full terminal covered in chars at independent random rates; two sinusoidal colour waves (different freq) wash across; 6 themes; 4 speed presets
- `polar_grid.c`  — polar/radial character grid: N_RINGS=10 concentric rings rotating at individual speeds (alternating dir); chars at ring-spoke intersections change randomly; colour fades inward→outward; 6 themes
- `hex_grid.c`    — hexagonal character grid: offset-row hex tiling; cube-coordinate ring distance from centre sets colour band; chars change at independent random rates; 6 themes
- `lissajous.c`   — Harmonograph/Lissajous figures: two damped perpendicular oscillators; parametric (x=A sin(aθ+δ), y=B sin(bθ)); decay and phase drift; multiple presets
- `spirograph.c`  — Spirograph: 3 simultaneous hypotrochoids with parameter drift, float canvas decay (FADE=0.985), 5 intensity levels, cyan/magenta/yellow curves
- `string_art.c`  — Mathematical string art: N=200 nails on circle, threads connect nail i→round(i×k) mod N, k drifts 2→7.5 morphing cardioid/nephroid/deltoid/astroid; rainbow thread colors; [/] nail count presets
- `voronoi.c`     — Animated Voronoi: 24 seeds with Langevin Brownian motion (DAMP=2, NOISE=60), brute-force O(N) nearest-seed per cell, d2−d1 edge detection, seed bounce off walls
- `convex_hull.c` — animated convex hull: Graham scan + Jarvis march running simultaneously; point scatter with force-directed placement; one step per frame; hull-edge draw with HUD comparison counts

### raster/
- `torus_raster.c`      — UV torus, 4 shaders (phong / toon / normals / wireframe), always-on back-face cull
- `cube_raster.c`       — unit cube, flat normals, same 4 shaders + toggleable cull + zoom
- `sphere_raster.c`     — UV sphere, same 4 shaders + toggleable cull + zoom
- `displace_raster.c`   — UV sphere with real-time vertex displacement (ripple / wave / pulse / spiky), central-difference normal recomputation

### raymarcher/
- `donut.c`             — parametric torus (no mesh): trigonometric projection, depth sort, luminance → grey pair
- `wireframe.c`         — wireframe cube via Bresenham 3-D projected edges, arrow-key rotation, slope-to-char line drawing
- `raymarcher.c`        — sphere-marching SDF raymarcher: sphere + plane, Blinn-Phong, gamma correction
- `raymarcher_cube.c`   — SDF box raymarcher: finite-difference normal, shadow ray
- `raymarcher_primitives.c` — multiple SDF primitives (sphere, box, torus, capsule, cone…) composited with `min`/`max`

### raytracing/
- `sphere_raytrace.c`   — quadratic ray-sphere intersection; orbiting camera; 3-point Phong; Fresnel glass mode; 256-color; 6 themes
- `cube_raytrace.c`     — AABB slab method; inverse-rotation ray transform; face-normal colour; pixel-perfect wireframe; 6 themes
- `torus_raytrace.c`    — quartic intersection via sampling+bisection; gradient normal; Fresnel; 6 themes
- `capsule_raytrace.c`  — cylinder body + hemisphere caps (IQ method); axial-projection body normal / sphere cap normal; inverse-rotation transform; 6 themes

---

## Core Architecture (all animation files)

### Coordinate / Physics Model
- Physics lives in **pixel space** — `CELL_W=8`, `CELL_H=16` sub-pixels per cell
- **One conversion point**: `px_to_cell_x/y()` in `scene_draw()` — nowhere else
- Simulations working in cell space (fire, sand, matrix_rain, flowfield) omit `§4 coords` entirely

### Simulation Loop
- Fixed-timestep accumulator: `sim_accum += dt; while (sim_accum >= tick_ns) { tick(); sim_accum -= tick_ns; }`
- `dt` capped at 100 ms to prevent spiral-of-death
- Render frame cap: **sleep BEFORE terminal I/O** — stable regardless of terminal write time
- `sim_fps` and render fps are independent

### Render Interpolation — alpha
- `alpha = sim_accum / tick_ns` ∈ [0.0, 1.0)
- Constant-velocity: forward extrapolation `draw_pos = pos + vel * alpha * dt`
- Non-linear forces: lerp `draw_pos = prev + (cur - prev) * alpha`

### ncurses Rendering
- Single `stdscr` — ncurses `curscr/newscr` IS the double buffer; no manual WINDOW pair
- Frame sequence: `erase() → draw scene → draw HUD → wnoutrefresh(stdscr) → doupdate()`
- `typeahead(-1)` — atomic diff write, no tearing
- `erase()` not `clear()` — no full-screen retransmit every frame

### Section Layout (§1–§8 or §1–§10 for complex files)
- `§1 config` — all tunable constants, `TICK_NS`, `CELL_W/H`
- `§2 clock` — `clock_ns()` (`CLOCK_MONOTONIC`) + `clock_sleep_ns()`
- `§3 color` — `color_init()`, 256-color with 8-color fallback
- `§4 coords` — `pw/ph/px_to_cell_x/y` (omitted in cell-space sims)
- `§5 physics` — simulation struct + tick function (no ncurses)
- `§6 scene` — owns physics objects; `scene_tick` + `scene_draw(alpha)`
- `§7 screen` — `screen_init/draw/present/resize`
- `§8 app` — `App` struct, `volatile sig_atomic_t` flags, main loop

### Signal Handling
- `SIGINT/SIGTERM` → `running = 0`; `SIGWINCH` → `need_resize = 1`
- Flags are `volatile sig_atomic_t` — compiler cannot cache, reads are atomic
- `atexit(cleanup)` calls `endwin()` — terminal always restored

---

## raster/*.c — Software Rasterizer Pipeline

```
tessellate_*()  →  scene_tick()  →  pipeline_draw_mesh()  →  fb_blit()
                   (rotate MVP)      for each triangle:
                                       vert shader  (VSIn → VSOut)
                                       clip/NDC/screen
                                       back-face cull
                                       rasterize (barycentric)
                                       z-test
                                       frag shader  (FSIn → FSOut)
                                       luma → Bayer dither → Bourke char → cbuf
```

- **Split `vert_uni / frag_uni`** in `ShaderProgram` — prevents segfault when vert and frag shaders need different uniform struct types
- **`cbuf[]`** intermediate framebuffer — rendering math is fully decoupled from ncurses I/O; `fb_blit()` is the sole boundary
- **`zbuf[]`** float depth buffer — initialised to `FLT_MAX`, z-test per cell

---

## Coding Conventions
- Comments explain **why** — especially non-obvious physics/rounding choices
- Every tunable constant in `§1 config` as enum or `#define`
- No dynamic allocation after init (except initial `malloc` in tessellate/flowfield/sand)
- `sig_atomic_t` for all signal-written flags
- `(chtype)(unsigned char)ch` double cast on every `mvaddch` — prevents sign-extension corruption
- C11, `-Wall -Wextra` clean

---

## Documentation

| File | Contents |
|---|---|
| `documentation/Architecture.md` | Framework design, loop mechanics, coordinate model, per-subsystem deep dives; §37 physics (lorenz/nbody/cloth), §38 artistic (aurora/voronoi/spirograph/plasma), §39 penrose tiling, §40 diamond-square terrain |
| `documentation/Master.md` | Long-form essays on algorithms, physics, and visual techniques; §P fractal systems, §Q artistic/signal effects (aurora/plasma/spirograph/voronoi), §R force-based physics (cloth/nbody/lorenz) |
| `documentation/Visual.md` | ncurses field guide — V1–V9 covering every ncurses technique (What/Why/How + code); Quick-Reference Matrix and Technique Index covering all files |
| `documentation/COLOR.md` | Color tricks across all C files — mechanism, exact code pattern, visual effect; includes fractal palettes, escape-time coloring, distance-based coloring, Buddhabrot density coloring |
