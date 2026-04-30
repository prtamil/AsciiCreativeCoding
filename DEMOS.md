# Demos

Per-program reference for every simulation in the repo. Each entry names the
algorithm, key data structures, and any non-obvious design choices. Programs
are grouped by topic; folder location is given at the start of each section.

For a high-level overview by folder, see the **Demos** section in
[README.md](README.md). For the underlying framework, see
[documentation/Architecture.md](documentation/Architecture.md).

Build any single program with:

```bash
gcc -std=c11 -O2 -Wall -Wextra <folder>/<file>.c -o <name> -lncurses -lm
```

---

## Fluid Dynamics  (`fluid/`)

| Program | Algorithm |
|---------|-----------|
| `lattice_gas` | FHP-I lattice gas — 6-direction bit-packed hex grid; 64-entry collision lookup (head-on 2-particle + symmetric 3-particle); streaming with bounce-back walls; momentum-colored display; 4 presets (cylinder/double-slit/channel/free), 5 themes |
| `navier_stokes` | Jos Stam stable fluid — Gauss-Seidel diffusion, semi-Lagrangian advection, divergence-free projection |
| `reaction_diffusion` | Gray-Scott model — 7 species presets (Mitosis, Coral, Stripes, Maze…) |
| `lenia` | Continuous Game of Life — smooth kernel convolution, organic moving creatures |
| `wave` | FDTD 2D wave PDE — CFL-stable, 5 interference sources, 4 color themes |
| `wave_2d` | 2D scalar wave PDE — Huygens interference, multiple point sources, signed amplitude colour map |
| `reaction_wave` | FitzHugh-Nagumo excitable medium — activator/inhibitor PDE, spiral waves, 4 color themes |
| `excitable` | Greenberg-Hastings N-state CA — resting/excited/refractory rule; 4 presets (Spiral/Double/Rings/Chaos); N adjustable 5–20 controls refractory depth and wave spacing; broken-front spiral nucleation; radially periodic IC for target rings; 5 themes; `spc` manual pulse |
| `wave_interference` | Analytic N-source wave interference — precomputed k·r phase table, aspect-corrected pixel distance, 8-level signed amplitude colour ramp; 4 presets (Double Slit/Ripple Tank/Beat/Radial); 5 themes; interactive source move/add/delete, ω and λ control |
| `fluid_sph` | SPH particle fluid — kernel density estimation, pressure + viscosity forces, symplectic Euler, O(N·k) spatial grid, 5 scenes, 8 themes |
| `flowfield` | Perlin fBm vector field — bilinear sampling, 8-direction particle trails |
| `complex_flowfield` | Complex-function vector field — conformal mapping, Joukowski transform, 6 field presets, 5 themes |
| `marching_squares` | Isosurface extraction — 16-case 4-bit lookup, linear edge interpolation, animated metaball scalar field |
| `sand` | Falling-sand cellular automaton — gravity, sliding, density sorting |
| `shallow_water_solver` | Shallow water equations — height + velocity fields, reflective boundaries, interactive wave injection |
| `vorticity_streamfunction_solver` | 2D incompressible flow — vorticity-streamfunction formulation, Jacobi-iterated Poisson solve, obstacle boundary |
| `cfl_stability_explorer` | CFL condition visualiser — live sweep of Courant number across FDTD schemes; shows stable vs unstable regimes |

## Physics  (`physics/`)

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
| `bubble_chamber` | Charged particles in a magnetic field — Lorentz force via exact velocity rotation R(ω), ionisation drag spirals orbits inward; 5 particle types (e⁻ e⁺ μ π p) with tuned q/m for visible curvature; age-faded trail ring buffers; `n` burst from centre, `e` burst from edge, `b/B` field strength, `Space` flip field, `t/T` particle type |
| `chain` | Hanging chain & swinging rope — Position-Based Dynamics (Verlet + iterative distance-constraint projection), tension-coloured links, 4 presets (Hanging/Pendulum/Bridge/Wave), trail ring-buffer, 5 themes |
| `rigid_body` | 2D rigid body physics — cubes + spheres, all pairs resolved with single AABB overlap function; spheres use aspect-corrected AABB `hw=r, hh=2r` to match terminal cell ratio; two-pass resolution: positional correction always fires (fixes overlap even when `vn=0`), velocity impulse only when approaching; adaptive restitution `e_eff=0` at low speed kills floor micro-bounce; spawn overlap check; sleep counter; `c` add cube, `s` add sphere, `x` remove last, `r` reset |
| `soft_body` | Jelly blob — 7×7 spring-mass mesh; structural + shear + bending springs (Hooke + velocity damping); Newtonian pressure from shoelace area vs target; scan-line fill rendering; symplectic Euler integration; 4 presets (Blob/Heavy/Bouncy/Two), 5 themes |
| `barnes_hut` | Barnes–Hut O(N log N) gravity — 800-body galaxy with quadtree force approximation (s/d < θ=0.5 criterion); static node pool (no malloc); flat rotation curve disk via M_enc∝r; Box-Muller Gaussian bulge; logarithmic spiral arms; brightness accumulator glow with DECAY=0.84; quadtree overlay (depth ≤ 3); 3 presets (Galaxy/Cluster/Binary), 5 themes |
| `gyroscope` | 3D rigid-body gyroscope — quaternion orientation (no gimbal lock), Gram-Schmidt re-ortho safeguard; 3 presets: Euler's Top (torque-free symmetric, angular momentum cone), Gravity Top (precession + nutation, wobble tightens with spin), Dzhanibekov (asymmetric torque-free, flip instability) |
| `spring_pendulum` | Spring pendulum — Lagrangian polar-coordinate EOM (r, θ); energy exchange resonance when ω_spring ≈ 2×ω_pendulum; rosette path tracing |
| `2stroke` | 2-stroke engine animation — slider-crank kinematics; crank/connecting-rod/piston geometry per θ; exhaust + transfer ports open/close; TDC spark; real-time cycle phase label |
| `nuke` | 2D shockwave demo — scalar wave PDE (∂²u/∂t² = c²∇²u − γ∂u/∂t) with 5-point Laplacian, CFL-stable substepping (CFL ≈ 0.33); cylindrical 1/√r decay + γ damping; terrain heave-and-settle ripples; debris arc + ground-dust pool; decaying sinusoidal screen shake; full-screen flash; 6 themes (`t` to cycle) |
| `beam_bending` | Euler-Bernoulli beam — 9 BC×load combos; analytical w(x) + M(x); curvature-shaded ASCII render + moment panel; dynamic modal superposition (4 eigenmodes, exact damped transition matrix) |
| `diff_drive_robot` | Differential drive robot — nonholonomic kinematics; pixel-space Euler integration; trail ring buffer; heading + wheel velocity arrows drawn with `.o0` dot progression |
| `acoustic_wavesolver` | Acoustic pressure wave solver — 2D FDTD on staggered pressure/velocity grid; absorbing PML boundary; interactive source placement |
| `lattice_boltzman_fluid_simulator` | Lattice Boltzmann fluid — D2Q9 BGK collision, streaming, bounce-back walls; density + velocity visualised; multiple obstacle presets |
| `mass_spring_lattice` | 2D mass-spring lattice — rectangular mesh of springs; symplectic Euler; wave packet injection; spring constant and damping tunable |
| `membrane` | 2D membrane vibration — FDTD wave equation on fixed-boundary grid; modal initialisation; aspect-correct pixel display |
| `conjugate_gradient_linear_solver` | Conjugate-gradient visualiser — animated convergence of CG solving Ax=b; residual norm display; comparison with Gauss-Seidel |
| `multigrid_solver_visualizer` | Multigrid solver — V-cycle Poisson solver; per-level residual animated; shows restriction and prolongation operators |
| `rk_method_comparision` | RK integrator comparison — RK1/2/4 side-by-side on same ODE; global error vs step-size; phase-space trajectories |
| `spectrogram_visualizer` | Spectrogram — real-time STFT with Hann window; frequency × time heat-map; 3-component sine mixer |
| `bounce_ball` | Reference implementation — single bouncing ball with gravity and floor restitution; the simplest complete framework example |

## Fractals & Chaos  (`fractal_random/`)

| Program | Algorithm |
|---------|-----------|
| `mandelbrot` / `julia` | Escape-time iteration, Fisher-Yates random reveal, 6 zoom presets |
| `julia_explorer` | Interactive Julia explorer — split screen: Mandelbrot map (left, precomputed) with crosshair, Julia set (right, per-frame recompute); arrow keys / HJKL move c; auto-wander orbits the Mandelbrot boundary; z/Z zoom Julia view; 5 themes |
| `barnsley` | IFS chaos game — 5 presets (Barnsley Fern/Sierpinski/Levy C/Dragon/Fractal Tree); probability-weighted affine transform selection; log-density hit accumulator rendered with 4 char levels; 5 themes |
| `diffusion_map` | Diffusion-Limited Aggregation — on-lattice random walk with launch circle + kill radius; Eden growth toggle (direct frontier pick); age-gradient coloring 5 levels; 5 themes (Coral/Ice/Lava/Plasma/Mono) |
| `tree_la` | Dielectric Breakdown Model — Gauss-Seidel Laplace relaxation, frontier growth probability ∝ φ^η; 3 presets (Tree/Lightning/Coral); live η control with `e`/`E` |
| `lyapunov` | Lyapunov fractal — alternating a/b logistic map sequences per pixel; λ sign determines stability (blue=stable, red=chaos); progressive row rendering; 6 sequences; 2 themes |
| `buddhabrot` | Two-pass orbit density accumulation, log-normalized nebula palette |
| `newton_fractal` | Complex Newton-Raphson on z⁴−1, basin coloring |
| `strange_attractor` | Clifford/de Jong/Ikeda density maps, log-normalized hit grid |
| `burning_ship` | Modified Mandelbrot with abs() trick |
| `bifurcation` | Logistic map, Feigenbaum period-doubling route to chaos |
| `apollonian` | Descartes' circle theorem, recursive circle packing |
| `l_system` | General L-system — 5 presets (Dragon Curve, Hilbert Curve, Sierpinski Arrow, Branching Plant, Koch Snowflake); string rewrite production rules; turtle graphics rendering; generation-by-generation growth animation |
| `lorenz` | Strange attractor — RK4, rotating projection |

## Cellular Automata & Life  (`misc/`, `fluid/`)

| Program | Algorithm |
|---------|-----------|
| `life` | Conway GoL + 5 rule variants (HighLife/Seeds/Day&Night…) |
| `cellular_automata_1d` | Wolfram 256 rules, 5 complexity classes color-coded |
| `langton` | Langton's ant + 7 multi-color turmite variants |
| `lenia` | Continuous CA — smooth kernel, real organisms |
| `hex_life` | Game of Life on hexagonal grid (6-neighbor offset layout) |
| `automaton_2d` | General 2D outer-totalistic CA with rule editing |

## Rendering & 3D  (`raster/`, `raymarcher/`)

| Program | Algorithm |
|---------|-----------|
| `raymarcher` | Sphere-marching SDF — Blinn-Phong, gamma correction |
| `raymarcher_cube` | SDF box — finite-difference normals, shadow ray |
| `raymarcher_primitives` | SDF boolean composition (min/max) — sphere/box/torus/capsule/cone |
| `sdf_gallery` | SDF composition gallery — 5 scenes: smooth-union blend, boolean ops, twist deformation, domain repetition, organic sculpt; 3 lighting modes (N·V / Phong / Flat); 5 themes |
| `mandelbulb_explorer` | 3D Mandelbulb raymarcher — spherical-power DE, tetrahedral normals, smooth-iter coloring, orbit traps, AO, soft shadows, progressive rendering, 2×2 supersampling toggle, 8 themes |
| `mandelbulb_raster` | Mandelbulb rasterizer — UV-sphere tessellation (~1800 triangles) built once at startup; MVP + z-buffer; per-vertex smooth-iter + normal; HSV fragment shaders; 4 shader modes (hue/normals/depth/Phong) |
| `torus_raster` | UV rasterizer — Phong/toon/normal/wireframe shaders, back-face cull |
| `cube_raster` / `sphere_raster` | Full software rasterizer pipeline |
| `displace_raster` | Real-time vertex displacement, central-difference normal recompute |
| `donut` | Parametric torus projection — the original spinning donut |
| `wireframe` | 3D Bresenham edge projection, slope-to-character line drawing |
| `sun` | 3D solar simulation — noise-displaced sphere SDF + domain-warped fBm boiling surface; 8 flares (blast → magnetic Bézier-arch → decay state machine, capsule SDFs smooth-unioned via smin); exponential corona accumulator; limb darkening (1-coefficient law); temperature-mapped 256-color palette; 4 themes |
| `nuke_v1` | Volumetric mushroom cloud — Beer–Lambert raymarched volume (no SDFs); single morphing anisotropic Gaussian blob (rx grows monotonic, ry grows-then-compresses) for fireball→cap; quintic smootherstep continuous-time morph; domain-warped fBm + value noise displacement; 2× vertical supersampling with sub-cell glyph picker; debris/ember/ash particles with air-drag terminal velocity; 5-second plateau then fall-collapse phase; 5 themes |

## Analytic Ray Tracing  (`raytracing/`)

| Program | Algorithm |
|---------|-----------|
| `sphere_raytrace` | Quadratic ray-sphere — orbiting camera, 3-point Phong, Fresnel glass mode, 256-color, 6 themes |
| `cube_raytrace` | AABB slab method — inverse-rotation ray transform, face-normal colour, pixel-perfect wireframe, 6 themes |
| `torus_raytrace` | Quartic intersection (sampling + bisection) — ring in XZ plane, gradient normal, Fresnel, 6 themes |
| `capsule_raytrace` | Cylinder + hemisphere caps — axial projection body normal, cap sphere normal, inverse-rotation transform, 6 themes |
| `path_tracer` | Monte Carlo path tracer — Lambertian BRDF, cosine hemisphere sampling (Malley's method), Russian roulette termination, progressive per-pixel accumulator, Reinhard tone map + gamma, Cornell Box scene with color bleeding |

## Emergent Systems  (`flocking/`)

| Program | Algorithm |
|---------|-----------|
| `flocking` | Reynolds boids — 5 modes (classic/leader/Vicsek/orbit/predator-prey) |
| `shepherd` | User-controlled herding — flee force, panic zone, flee-radius ring |
| `crowd` | Reynolds steering crowd — 6 live-switchable behaviours (WANDER/FLOCK/PANIC/GATHER/FOLLOW/QUEUE); up to 150 agents; seek/flee/separate/align/cohesion forces |
| `war` | Two-faction battle (GONDOR vs MORDOR) — melee + archer units; travelling `-` arrow projectiles (flat pool, 220 px/s); 4-state FSM (ADVANCE/COMBAT/FLEE/DEAD); 6 live battle strategies |
| `swarm_gen_numbers` | Reynolds steering digit swarm — 25 agents form digits 0–9 via 10 strategies (DRIFT/RUSH/FLOW/ORBIT/FLOCK/PULSE/VORTEX/GRAVITY/SPRING/WAVE); greedy slot assignment; Hooke's law spring steering |
| `ant_colony` | Pheromone ACO — stigmergic path optimization |
| `wator` | Wa-Tor predator-prey ecosystem |
| `network_sim` | SIR epidemic + spring-force graph layout |
| `slime_mold` | Physarum polycephalum — Jeff Jones (2010) agent model; 3-sensor sense→rotate→move→deposit loop; double-buffered trail diffusion + decay; emergent minimum Steiner tree networks connecting food sources; 4 presets, 5 themes |

## Turtle Graphics  (`turtle/`)

| Program | Algorithm |
|---------|-----------|
| `duo_poly` | Dual turtle polygon animator — two turtles (cyan A / magenta B) draw regular polygons step-by-step; each tick advances one edge; aspect-corrected Y keeps shapes visually round; auto-cycles 3→12 sides; `a/z` `s/x` sides, `+/-` speed |

## Grid Systems  (`grids/`)

All major grid families are split into two parallel sub-folders: a **display**
folder showing the bare grid, and a **placement** folder where a cursor can
deposit objects on that grid. Eight families: rectangular (14 displays + 4
unified placement editors), polar (7 + 4), hex (7 + 4), and triangular (12 + 24).

### Background Grid Displays  (`grids/rect_grids/`)

| Program | Grid Type |
|---------|-----------|
| `01_uniform_rect` | Regular rectangular grid — `+` junctions, `-` rows, `\|` cols |
| `02_square` | Square cells — equal visual proportions via `SQ_CS×2` / `SQ_CS` |
| `03_fine_dense` | Fine dense grid — small `4×2` cells, high line density |
| `04_coarse_sparse` | Coarse sparse grid — large `12×4` cells, open space |
| `05_hierarchical` | Three-weight hierarchy — major `#=`, semi `\|-`, minor `.:` lines |
| `06_brick_stagger` | Horizontal brick — even rows shifted right by `cw/2` |
| `07_half_brick_vert` | Vertical brick — even columns shifted down by `ch/2` |
| `08_diamond` | Diamond grid — 45° rotated with `/\` line chars |
| `09_isometric` | Isometric 2:1 oblique projection — `/\` at 2:1 aspect |
| `10_crosshatch` | Crosshatch — rectangular grid + 45° diagonal overlay |
| `11_checkerboard` | Checkerboard — alternating `#`-filled squares |
| `12_ruled` | Ruled — horizontal lines only, `RL_LS=3` line spacing |
| `13_dot` | Dot grid — `*` at intersections only |
| `14_origin` | Origin-marked grid — `=` x-axis, `I` y-axis, `+` at crossing |

### Interactive Placement Editors  (`grids/rect_grids_placement/`)

| Program | Algorithm |
|---------|-----------|
| `01_direct` | Cursor placement — arrow-key navigation, `space` toggles objects; `GridCtx` abstraction drives all 14 grid types from one cursor |
| `02_patterns` | Pattern stamp — 5 predicates (border/fill/hollow/row/col) stamped at cursor; live preview; `+/-` resizes |
| `03_path` | Two-point path drawing — `p` cycles IDLE→A→B; `l`=Bresenham line, `j`=L-path, `o`=ring, `x`=diagonal |
| `04_scatter` | Procedural scatter — `R`=random, `M`=Poisson min-distance, `F`=BFS flood, `G`=gradient density |

### Polar Grid Displays  (`grids/polar_grids/`)

| Program | Algorithm |
|---------|-----------|
| `01_rings_spokes` | Standard polar grid — concentric rings + radial spokes; fmod detects all simultaneously |
| `02_log_polar` | Log-polar grid — rings at `R_MIN × RATIO^k`; fractional width in log-ring-index space |
| `03_archimedean_spiral` | Archimedean spiral — constant-pitch arms; N-arm phase test `fmod(N×(θ−r/a), 2π)` |
| `04_log_spiral` | Logarithmic spiral — gap grows with radius; golden spiral preset `a≈0.3065` |
| `05_sunflower` | Phyllotaxis — Vogel model `(√i×spacing, i×GOLDEN_ANGLE)`; 'g' cycles angle variants |
| `06_sector` | Equal-area sectors — rings at `√k × R_UNIT` so every annulus has equal area |
| `07_elliptic` | Elliptic polar — `e_r = sqrt((dx/A)²+(dy/B)²)`; 'h' overlays confocal hyperbolae |

### Polar Placement Editors  (`grids/polar_grids_placement/`)

| Program | Algorithm |
|---------|-----------|
| `01_polar_direct` | Cursor placement — screen-mode (Δrow/Δcol) or polar-mode (Δr/Δθ); `m` toggles; all 7 polar backgrounds via `a`/`e` |
| `02_polar_arc` | Arc/spoke/ring drawing — two-anchor state machine; `l`=arc, `s`=spoke, `r`=ring, `x`=radial; PAIR_ANCHOR highlights anchors |
| `03_polar_spiral` | Parametric spiral placement — `l`=Archimedean `r=r₀+aθ`, `o`=log-spiral `r=r₀eᵍᶿ`; `d` draws; pitch/turns/density tunable |
| `04_polar_scatter` | Procedural scatter — `U`=uniform-area, `G`=radial-Gaussian (Box-Muller), `W`=wedge, `D`=ring-snap; `[`/`]` adjusts sigma/wedge |

### Hex Grid Displays  (`grids/hex_grids/`)

| Program | Grid Type |
|---------|-----------|
| `01_flat_top` | Flat-top hexagonal grid — forward matrix `cx=size×3/2×Q`, `cy=size×(√3/2×Q+√3×R)`; cube-round hit test; angle_char border rasterizer |
| `02_pointy_top` | Pointy-top hexagonal grid — rotated forward matrix; alternating row offset layout; same axial core |
| `03_axial` | Axial coordinate display — Q/R/S axis lines; cube constraint Q+R+S=0 visualised; ring-distance colour bands |
| `04_ring_distance` | Ring-distance colouring — hex_dist=(|dQ|+|dR|+|dQ+dR|)/2; concentric colour rings from origin |
| `05_triangular` | Triangular-dual grid — triangular tessellation derived from hex centres; up/down triangle parity |
| `06_rhombille` | Rhombille tiling — three-direction diamond lattice; cube-face projection mapping |
| `07_trihexagonal` | Trihexagonal (Kagome) tiling — alternating hexagons and triangles; vertex-figure `3.6.3.6` |

### Hex Placement Editors  (`grids/hex_grids_placement/`)

| Program | Algorithm |
|---------|-----------|
| `01_hex_direct` | Cursor placement — axial (Q,R) cursor; `space` toggles objects; `+/-` hex size; cube_round hit-test; HEX_DIR[4] movement |
| `02_hex_pattern` | Pattern stamp — disc/ring/row/col predicates in axial space; full per-pixel border overlay preview (bright green); `+/-` radius; 4 stamp glyphs |
| `03_hex_path` | Two-endpoint path drawing — `a`/`b` set endpoints; `l`=line (hex_lerp_round), `o`=ring (6N cells), `j`=L-path; live green-dot preview |
| `04_hex_scatter` | Procedural scatter — `1`=uniform, `2`=min-dist (hex_dist rejection), `3`=flood-fill disc, `4`=gradient density; `+/-` radius |

### Triangular Grid Displays  (`grids/tri_grids/`)

Twelve triangular tilings, ranging from regular periodic grids (1–6) through
recursive fractals (7–9), aperiodic substitutions (10, 12), and irregular
mesh (11). Cursor demos for the regular tilings; depth control for the
fractals; cycle-through-triangles for Delaunay.

| Program | Grid Type |
|---------|-----------|
| `01_equilateral` | Equilateral triangular tiling via 2-axis skew lattice (v₁=(s,0), v₂=(s/2, s√3/2)); ▽/△ pair per rhombus split by fa+fb=1; barycentric edge picker; cursor (col, row, up) walked by TRI_DIR[4][2] |
| `02_right_isosceles` | Half-rect tiling — square cells split by single `\` diagonal into UR/LL right-isosceles triangles; axis-aligned (no shear) lattice; barycentric weights pick edge char |
| `03_double_diagonal` | Tetrakis square — both diagonals split each square into 4 right-isosceles wedges (N/E/S/W apex); wedge classifier from \|Δx\| vs \|Δy\|; vertex config 8.8.8.8 |
| `04_30_60_90` | Kisrhombille — equilateral grid with three medians per triangle; 6 right (30-60-90) sub-triangles per equilateral; signed-distance line equations for medians |
| `05_isometric` | Same lattice as 01, rendered as solid colored blocks; (col + 2·row + up) mod 6 hash gives stacked-cube isometric pattern |
| `06_hex_subdivision` | Flat-top hex grid + 3 long diagonals through each centre, splitting each hex into 6 equilateral wedges; sector classifier via atan2 from hex centre |
| `07_barycentric` | Recursive 6-way subdivision — centroid + 3 midpoints per triangle; 6^N leaves; Hatcher §2.1 algebraic-topology decomposition |
| `08_triforce` | Recursive 4-way midpoint subdivision (3 corners + inverted centre); 4^N leaves; Loop subdivision base step |
| `09_sierpinski` | Same as triforce but center child dropped — 3^N leaves, Hausdorff dim log₂3 ≈ 1.585; classic gasket |
| `10_pinwheel` | Pinwheel-inspired 5-way split of a 1-2-√5 right triangle (4 midpoint corners + 2 altitude halves of inverted centre); aperiodic-looking; documented deviation from strict Conway-Radin |
| `11_delaunay` | Delaunay triangulation of N random points via Bowyer-Watson incremental insertion; super-triangle scaffold; in-circumcircle determinant predicate; cycle cursor through triangles |
| `12_penrose` | Robinson-triangle substitution — acute (apex 36°, leg=φ·base) + obtuse (apex 108°, base=φ·leg); golden-ratio φ = (1+√5)/2 split per step; A→A+B and B→A+B; 2^N leaves; aperiodic in the limit |

### Triangular Placement Editors  (`grids/tri_grids_placement/`)

24 placement programs: 6 regular tri-grid types × 4 placement variants
(direct / patterns / path / scatter). Cursor lives at `(col, row, up)` for
01/02/04/05, `(col, row, dir)` for 03 (N/E/S/W), and `(Q, R, sector)` for 06.

| Program | Algorithm |
|---------|-----------|
| `01_equilateral_direct` | Direct placement on equilateral grid — arrow keys move (col, row, up) via TRI_DIR[4][2]; `space` toggles glyph at cursor; `g` cycles glyphs; ObjectPool |
| `01_equilateral_patterns` | Preset stamps on equilateral — `1`=ring `2`=line `3`=star `4`=triforce `5`=10-tri scatter; (Δcol, Δrow, target_up) tables |
| `01_equilateral_path` | Line-of-sight path — `s`/`e` set start/end; pixel-walk between centroids samples every size·0.25 px; recorded (col, row, up) triples form the path |
| `01_equilateral_scatter` | Random scatter colored by Manhattan-style cell distance from cursor; 6-stop gradient palette; `+/-` density 20–500, SPACE reseeds |
| `02_right_isosceles_direct` | Direct placement on half-rect grid — UR/LL toggle; same pool/cursor pattern as 01 with axis-aligned lattice |
| `02_right_isosceles_patterns` | Preset stamps on half-rect grid — same pattern set as 01 with UR/LL orientations |
| `02_right_isosceles_path` | Line-of-sight path on half-rect grid |
| `02_right_isosceles_scatter` | Distance-colored scatter on half-rect grid |
| `03_double_diagonal_direct` | Direct placement on tetrakis — `(col, row, dir)` cursor with TETRA_DIR[4][4]; arrows move N/E/S/W toward compass within square |
| `03_double_diagonal_patterns` | Preset stamps on tetrakis — RING (all 4 wedges of cursor), LINE (8 wedges across 2 squares), STAR, TRI |
| `03_double_diagonal_path` | Line-of-sight path on tetrakis |
| `03_double_diagonal_scatter` | Distance-colored scatter on tetrakis; distance metric includes \|Δdir\| (mod 4) |
| `04_30_60_90_direct` | Direct placement on kisrhombille — same cursor as 01 (whole equilaterals); background renders both equilateral edges and 3 medians per triangle |
| `04_30_60_90_patterns` | Preset stamps on kisrhombille |
| `04_30_60_90_path` | Line-of-sight path on kisrhombille |
| `04_30_60_90_scatter` | Distance-colored scatter on kisrhombille |
| `05_isometric_direct` | Direct placement on iso (solid-fill) grid — palette_index hash colors triangles in 6-cycle; objects render in reverse-video over fill |
| `05_isometric_patterns` | Preset stamps on iso grid |
| `05_isometric_path` | Line-of-sight path on iso grid (edge-rendered for visibility) |
| `05_isometric_scatter` | Distance-colored scatter on iso grid |
| `06_hex_subdivision_direct` | Direct placement on hex with 6 wedges — `(Q, R, sector)` cursor; arrows walk hex (HEX_DIR[4]); `,`/`.` rotate sector; `space` toggles |
| `06_hex_subdivision_patterns` | Preset stamps on hex-subdivision — RING (6 sectors of cursor hex), LINE (sector 0 across 6 hexes), STAR, TRI |
| `06_hex_subdivision_path` | Line-of-sight path on hex-subdivision; pixel walk + sector_of() classifier per sample |
| `06_hex_subdivision_scatter` | Distance-colored scatter on hex-subdivision; cube-distance + sector-circular distance |

## Geometry  (`geometry/`)

| Program | Algorithm |
|---------|-----------|
| `rect_grid` | Rectangular character grid — per-cell random rate, dual sinusoidal colour waves, 6 themes |
| `polar_grid` | Polar character grid — concentric rings, alternating rotation, colour by ring, 6 themes |
| `hex_grid` | Hexagonal character grid — offset-row tiling, cube-coordinate ring distance for colour bands |
| `grid_proper` | Unified grid explorer — all grid types in a single interactive switcher |
| `lissajous` | Harmonograph/Lissajous — two damped perpendicular oscillators, phase drift |
| `spirograph` | Hypotrochoid parametric curves — 3 simultaneous with parameter drift |
| `string_art` | Modular arithmetic i→⌊i×k⌋ mod N, morphing cardioid/nephroid/astroid |
| `voronoi` | Brute-force nearest-neighbor, Langevin seed motion, d2−d1 edge detection |
| `convex_hull` | Graham scan + Jarvis march — simultaneous race |
| `delaunay_triangulation` | Bowyer-Watson incremental Delaunay — circumcircle insertion, edge flip to fix violations |
| `kd_tree` | k-d tree — 2D median-split BSP; nearest-neighbour and range-query animated; static pool |
| `visibility_polygon` | Visibility polygon — radial sweep, endpoint sort, shadow casting from a point light |
| `quad_tree_helloworld` | Animated quadtree — INSERT phase (random points, live subdivision) → QUERY phase (drifting rectangle, AABB pruning visible); static node pool, depth-coloured borders, scrolling info panel |
| `quadtree` | Quadtree pure-C demo — 8-step walkthrough: fill root, trigger VERTICAL+HORIZONTAL subdivisions, range query; ANSI-coloured ASCII grid; malloc-based nodes |
| `bsp_tree` | BSP tree pure-C demo — alternating VERTICAL (!) / HORIZONTAL (=) splits, front/back children, AABB range query with full right-half pruning; 8-step walkthrough |

## Mathematical Art  (`artistic/`)

| Program | Algorithm |
|---------|-----------|
| `epicycles` | DFT epicycles — sorted-by-amplitude arm chain, convergence animation |
| `fourier_art` | User-drawn path → Fourier reconstruction — draw any shape with cursor keys, arc-length resample to 256 pts, O(N²) DFT, epicycle arm chain replay with auto-add convergence, 5 themes |
| `fft_vis` | Cooley-Tukey FFT visualiser — bit-reversal permutation, radix-2 butterfly O(N log N), live time-domain + frequency-domain dual panel, twiddle factors W_N^k = exp(−2πik/N), 3-component sine mixer |
| `cymatics` | Chladni figures — 2D standing wave nodal lines, 20 modes |
| `plasma` | Demoscene: 4-component sin-sum, palette cycling |
| `aurora` | Multi-octave sinusoidal curtains, deterministic star hash |
| `penrose` | de Bruijn pentagrid duality — aperiodic tiling, slow rotation |
| `terrain` | Diamond-square heightmap — thermal erosion, 7 contour levels |
| `perlin_landscape` | Perlin fBm — 3 parallax terrain layers, 5-octave noise, painter's algorithm |

## Animation & Kinematics  (`animation/`, `robots/`)

| Program | Algorithm |
|---------|-----------|
| `hexpod_tripod` | 6-legged robot — tripod gait (alternating support triangles), 2-joint analytical IK (law of cosines), 4-direction steering with angular interpolation, toroidal wrap |
| `ik_spider` | IK spider — sinusoidal body locomotion, 2-joint IK per limb, step-trigger gait |
| `ik_arm_reach` | 2-joint arm — FABRIK IK reach with elbow-side toggle, Lissajous target path |
| `ik_tentacle_seek` | Seeking tentacles — FABRIK solver on wandering target, per-segment reach tolerance, multiple independent chains |
| `ragdoll_figure` | Ragdoll stick figure — constraint-projected Verlet joints, momentum carry-over |
| `ragdoll_ropes` | Multi-rope Verlet chains — damping, phase-offset anchors, iterative constraint relaxation |
| `snake_forward_kinematics` | FK snake — circular trail-buffer chain, sinusoidal heading, bead rendering, 10 themes |
| `snake_inverse_kinematics` | FABRIK inverse kinematics snake — iterative forward/backward reach solver |
| `fk_centipede` | Centipede — trail-buffer FK body, stateless sinusoidal FK legs, contralateral antiphase gait |
| `fk_tentacle_forest` | Tentacle forest — pure stateless sinusoidal FK; per-tentacle phase/frequency/amplitude parameters |
| `fk_medusa` | Medusa jellyfish — radial bell-oscillation FK + cascaded trailing tentacle FK chains |
| `walking_robot` | Procedural bipedal walk cycle — sinusoidal FK, 2-joint analytical IK stance, foot contact locking, body sway, shadow ellipse, COM projection, motion trails, ground grid |
| `moving_jump_spring_leg_robot` | Spring-leg jumping robot — spring-mass leg compression/release, aerial phase, landing absorption |
| `perlin_terrain_bot` | Self-balancing wheel bot — inverted pendulum Lagrangian cart-pole on Perlin terrain slope; PID controller with cascade slope feed-forward; phase portrait, gain preset tuning |

## Artistic / Biological  (`artistic/`, `matrix_rain/`)

| Program | Algorithm |
|---------|-----------|
| `galaxy` | Spiral galaxy — 3000 stars in circular orbits with flat rotation curve (ω = v₀/r); logarithmic spiral arm initialization; brightness accumulator grid with per-frame decay creates natural trails; normalised density → char (`.,:oO0@`); radial colour zones (core/disk/halo); 2–4 arms, 5 themes |
| `jellyfish` | Physics pulse locomotion — IDLE sink → CONTRACT jet → GLIDE coast → EXPAND bloom; asymmetric bell (width × height axes); tentacle inertia lag |
| `xrayswarm` | Multi-swarm radial pulse — DIVERGE → PAUSE → CONVERGE; workers park at screen edge, retrace exact origin path; 4-pass rendering prevents trail cancellation |
| `gear` | Wireframe rotating gear — proximity-based edge detection, tangential surface-velocity sparks, speed-proportional emission, 10 color themes (fire/matrix/plasma/nova/poison/ocean/gold/neon/arctic/lava) |
| `railwaymap` | Procedural transit map — H/V/Z grid-aligned line templates, canvas-based ACS junction detection, station interchange, 10 themes |
| `fireworks_rain` | Fireworks with matrix-rain arc trails — each of 72 sparks per explosion grows a 16-slot position-history trail; chars shimmer 75 % per tick; 5 themes (vivid/matrix/fire/ice/plasma) remap all spark color pairs; `t` cycles theme |
| `matrix_rain` | Classic digital rain — cascading katakana/Latin glyphs, speed and density tunable, 5 themes |
| `matrix_snowflake` | Matrix rain + live DLA snowflake — two real simulations on one screen: classic digital rain in the background; a D6-symmetric DLA ice crystal grows from the center in the foreground, freezing 12 symmetric positions per walker stick event; crystal flashes white on completion then resets; 5 themes (Classic/Inferno/Nebula/Toxic/Gold) |
| `pulsar_rain` | Rotating pulsar neutron star — N-beam (1–16) lighthouse sweep; angular wake cache; pre-computed trig reuse across radii; render interpolation for smooth rotation |
| `sun_rain` | Matrix-rain solar — circular clip mask, radial solar wind streams, parametric border ring with rotating wave |
| `led_number_morph` | Particle digit morphing — 168 particles form a scaled 7-segment LED display; particles belong permanently to one segment and spring to their targets when the segment is lit, drift to centre when dark; orientation-aware chars ('-' horizontal, '|' vertical) for formed segments; scales with terminal height; 5 themes with per-digit colours; `n` skip, `]`/`[` speed |
| `particle_number_morph` | Solid filled particle morphing — up to 500 particles densely pack the full interior of a 9×7 bitmap font digit; greedy nearest-neighbour matching routes every particle to its closest target; positions lerp with smoothstep easing (no spring/velocity) for a clean deterministic glide; idle particles glide to centre and vanish; `f`/`F` morph speed, `]`/`[` hold time; 5 themes |
| `dune_rocket` | Dune-universe rocket launch — particle exhaust trails, `+/-` launch rate, `Space` salvo burst |
| `dune_sandworm` | Dune sandworm — sinusoidal body locomotion, surface breach animation, `Space` trigger breach, `+/-` speed |
| `sand_art` | Hourglass sand art — 5-layer coloured falling-sand CA; gravity-flip on `Space`; scan sweeps away from gravity so grains never move twice per tick; `R` pour fresh layers |
| `bat` | Bat silhouette animation — flapping wing kinematics, Bézier curve body outline, moth-hunt targeting |
| `bonsai` | Procedural bonsai tree — recursive L-system branching, aging simulation, seasonal cycle |
| `leaf_fall` | Falling leaves — Euler-angle tumbling with aerodynamic torque, ground accumulation |
| `dna` | DNA double helix — parametric strand animation, base-pair rungs, rotation and colour cycling |

## Particle Systems  (`particle_systems/`)

| Program | Algorithm |
|---------|-----------|
| `fire` | Fire — 3 switchable algorithms: CA heat diffusion, upward particle splat, plasma sine sum; shared Floyd-Steinberg + LUT pipeline |
| `smoke` | Smoke — 3 algorithms: CA diffusion, particle Gaussian splat, Perlin-driven curl; shared density→char pipeline |
| `fireworks` | Fireworks — 3-state rocket FSM (LAUNCH/BURST/FALL); shell burst to N sparks; gravity + drag |
| `kaboom` | Explosion — deterministic LCG ring expansion, shock-front char ramp, debris scatter |
| `particles` | General particle sandbox — spawn, gravity, bounce, colour-by-age |
| `aafire_port` | AAlib fire port — classic fire algorithm adapted for ncurses; bottom-row heat source, upward diffusion, char-density palette |
| `brust` | Staggered burst waves — multiple expanding rings with phase offset, echo decay |
| `constellation` | Constellation network — proximity-linked stars, Delaunay-style edge pruning, slow drift |

## Algorithms  (`misc/`)

| Program | Algorithm |
|---------|-----------|
| `sort_vis` | 5 sort algorithms — animated bar chart, comparison+swap counters |
| `maze` | DFS generation + BFS/A* animated solve |
| `graph_search` | BFS/DFS/A* on grid — animated frontier expansion |
| `forest_fire` | Drossel-Schwabl CA — 3-state (EMPTY/TREE/FIRE) probabilistic update; neighbour-spread + lightning ignition; ratio p/f controls cluster size and self-organised criticality; 4-way/8-way spread, 4 presets, 5 themes |
