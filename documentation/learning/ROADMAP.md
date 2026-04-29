# Learning Roadmap — ASCII Creative Coding

221 C files, 40+ topics. This roadmap gives the optimal study order, the per-file loop,
and the 2-year breakdown. Do not skip layers.

---

## The Learning Loop (do this for every single file)

```
1. MATH FIRST  — find the original paper/algorithm, understand the equation on paper
2. READ CODE   — map every code block to the equation it implements
3. ANNOTATE    — rewrite comments in your own words explaining WHY not WHAT
4. BREAK IT    — change one constant at a time, predict before running
5. REWRITE     — close the file, rewrite from scratch using only the math source
6. VARY        — add one thing the original did not do (this is where it becomes yours)
```

**The single most important rule:** Never run the program before you can predict its output.

---

## Tier 1 — Visual Math (2–3 months)
Needs only geometry and trigonometry. Fast feedback loops.

| File | Subject | Key Math |
|---|---|---|
| `cellular_automata_1d.c` | Wolfram CA | bitwise rule lookup |
| `life.c` | Conway GoL | Moore neighborhood |
| `langton.c` | Turmite | finite state machine |
| `sierpinski.c` | IFS chaos game | affine transforms |
| `fern.c` | Barnsley IFS | 4-transform probability |
| `koch.c` | L-system | recursive subdivision |
| `dragon_curve.c` | L-system | iterative string rewrite |
| `l_system.c` | General L-system | production rules |
| `duo_poly.c` | Turtle graphics — dual polygon animator | regular polygon vertex angles θ_k = θ₀ + k·(2π/n), aspect-corrected Y, DDA segment fill, edge timer |
| `lissajous.c` | Parametric curves | sin/cos phase |
| `spirograph.c` | Hypotrochoid | parametric gear math |
| `string_art.c` | Modular arithmetic | i → round(i×k) mod N |
| `epicycles.c` | Fourier series | DFT, complex exponentials |
| `automaton_2d.c` | 2D CA | 2D neighborhood rules |
| `hex_life.c` | Hex grid CA | offset-row coordinate mapping |
| `aurora.c` | Sinusoidal curtains | multi-octave sin |
| `cymatics.c` | Chladni figures | 2D standing wave nodes |
| `xrayswarm.c` | Multi-swarm pulse locomotion | radial velocity, converge/diverge state machine |
| `jellyfish.c` | Physics-based pulse locomotion | jet propulsion, exponential drag, asymmetric bell deformation |
| `gear.c` | Wireframe gear + themed sparks | polar proximity edge detection, tangential surface velocity, 256-color theme switching |
| `pulsar_rain.c` | Rotating pulsar neutron star | N-beam lighthouse sweep (1–16), angular wake cache, pre-computed trig reuse across radii, render interpolation |
| `sun_rain.c` | Matrix rain sun | circular clip mask, radial solar wind streams, parametric border ring with rotating wave |
| `railwaymap.c` | Procedural transit map | H/V/Z line templates on 6×4 logical grid, canvas ACS junction detection, interchange nodes |
| `fireworks_rain.c` | Fireworks + matrix arc trails | position-history trail, oldest-first draw, split ROCKET_DRAG/GRAVITY, per-particle gravity variance, 5-theme color pair remapping |
| `matrix_snowflake.c` | Matrix rain + live DLA crystal | two-simulation layering (rain background / DLA foreground), D6 symmetry with aspect correction, proximity-spawn DLA optimisation, flash/reset lifecycle, 5 themes |
| `fourier_art.c` | User-drawn path → Fourier epicycles | DRAW→PLAY state machine, arc-length resampling, O(N²) DFT, amplitude-sorted epicycle chain, auto-add convergence |
| `galaxy.c` | Spiral galaxy — differential rotation | flat rotation curve ω=v₀/r, logarithmic spiral arm init, brightness accumulator + frame decay, radial colour zones |
| `fft_vis.c` | Cooley-Tukey FFT visualiser | bit-reversal permutation, radix-2 butterfly O(N log N), time vs frequency panel, twiddle factors W_N^k = exp(-2πik/N) |
| `01_uniform_rect.c` … `14_origin.c` | 14 rectangular grid displays — cell-space rendering | direct screen-sweep: `(sr%ch==0 \|\| sc%cw==0)`, brick stagger `(r%2)*(cw/2)`, diamond `safe_mod(u*IH+v*IW, 2*IW*IH)`, iso 2:1 oblique |
| `01_direct.c` | Direct cursor placement — GridCtx unification | `ctx_to_screen` as single coordinate seam, ObjectPool swap-last, grid cycling `(mode±1+GM_COUNT)%GM_COUNT` |
| `02_patterns.c` | Pattern stamp — predicate-driven fill | border/fill/hollow/row/col as boolean functions of `(dr,dc,N)`; preview overlay before stamp |
| `03_path.c` | Two-point path drawing | Bresenham line (error accumulation), L-path, ring border, diagonal staircase; 3-state `SEL_IDLE→ONE→TWO` FSM |
| `04_scatter.c` | Procedural object scatter | random uniform, Poisson-disk rejection sampling, BFS flood fill, gradient Bernoulli trials; Chebyshev distance |
| `01_rings_spokes.c` | Standard polar grid — rings + spokes | `cell_to_polar` with CELL_H/CELL_W aspect correction; fmod ring + spoke test; `angle_char` tangent character |
| `02_log_polar.c` | Log-polar grid — exponential ring spacing | `u = log(r/R_MIN)/log_step`; fractional width `RING_W_U` in log-index space; log-polar transform in retina/SIFT |
| `03_archimedean_spiral.c` | Archimedean spiral — constant pitch | N-arm phase test `fmod(N×(θ−r/a)+N×2π, 2π)`; why all N arms map to phase≈0 |
| `04_log_spiral.c` | Logarithmic spiral — growing pitch | `θ_pred = log(r/b)/a`; golden spiral `a=2ln(φ)/π≈0.3065`; equiangular property |
| `05_sunflower.c` | Phyllotaxis — Vogel sunflower | `(√i×spacing, i×GOLDEN_ANGLE)`; equal-area √i spacing; GOLDEN_ANGLE=2π/φ²; parametric vs screen-sweep |
| `06_sector.c` | Equal-area sector grid | `k_float=(r/R_UNIT)²`; annular area constant; HEALPix connection |
| `07_elliptic.c` | Elliptic polar + confocal hyperbolae | `e_r=sqrt((dx/A)²+(dy/B)²)`; `ell_theta` for tangent chars; confocal conic orthogonality |
| `01_polar_direct.c` | Polar cursor placement — dual-representation | `Cursor{r,θ,row,col}`; screen↔polar mode sync; `polar_to_screen` inverse: `col=ox+round(r×cos(θ)/CELL_W)` |
| `02_polar_arc.c` | Arc/spoke/ring drawing — two-anchor FSM | IDLE→ONE→TWO state machine; arc step `CELL_W/(r+1)` adapts to radius; ring = full 0→2π walk |
| `03_polar_spiral.c` | Parametric spiral placement | Archimedean `r=r₀+a×t`; log `r=r₀×e^(growth×t)`; parametric walk vs screen-sweep; golden spiral preset |
| `04_polar_scatter.c` | Polar scatter — 4 strategies | uniform-area `r=sqrt(r₀²+rand×(r₁²-r₀²))`; Box-Muller radial-Gaussian; wedge angular clamp; ring-snap quantisation |
| `01_flat_top.c` | Flat-top hexagonal grid — axial coordinates, forward matrix | forward: `cx=size×3/2×Q`, `cy=size×(√3/2×Q+√3×R)`; inverse: `fq=(2/3×px)/size`, `fr=(-px/3+√3py/3)/size`; cube_round for hit-test |
| `02_pointy_top.c` | Pointy-top hexagonal grid — rotated orientation | rotated forward matrix; both orientations share same axial coordinate engine |
| `03_axial.c` | Axial coordinate display — Q/R/S axes, cube constraint | Q+R+S=0 plane visualised; axial vs cube vs offset coordinate systems |
| `04_ring_distance.c` | Ring-distance colouring | `hex_dist=(|dQ|+|dR|+|dQ+dR|)/2`; concentric colour rings from any centre |
| `05_triangular.c` | Triangular-dual grid — hex dual graph | hex centres become triangle vertices; parity `(row+col)%2` selects up/down triangle |
| `06_rhombille.c` | Rhombille tiling — cube-face diamonds | three cube faces projected onto 2D; each rhombus is one cube face |
| `07_trihexagonal.c` | Trihexagonal (Kagome) tiling — `3.6.3.6` vertex figure | alternating triangle+hexagon rings; one of the 11 Archimedean tilings |
| `01_hex_direct.c` | Hex cursor placement | axial cursor + HEX_DIR[4] movement; swap-last O(1) object pool; cube_round inverse map |
| `02_hex_pattern.c` | Hex pattern stamp — predicate-driven fill | disc/ring/row/col predicates on `(dQ,dR)`; full per-pixel overlay rasterizer; shows preview before stamp |
| `03_hex_path.c` | Hex path drawing — line/ring/L-path | hex_lerp_round for interpolated line; ring walk 6×N steps; `a`/`b` dual-endpoint FSM |
| `04_hex_scatter.c` | Hex procedural scatter — 4 strategies | uniform density; hex_dist min-distance rejection; BFS flood disc; gradient Bernoulli `P=k/(d+k)` |

---

## Tier 2 — Simulation Basics (2–3 months)
Introduces differential equations and numerical integration.

| File | Subject | Key Math |
|---|---|---|
| `bounce_ball.c` | Newtonian motion | Euler integration |
| `spring_pendulum.c` | Lagrangian mech | Lagrange equations |
| `double_pendulum.c` | Chaos | RK4 integration |
| `pendulum_wave.c` | Phase arithmetic | ω_n = 2π(N+n)/T |
| `lorenz.c` | Strange attractor | RK4, Lyapunov exponents |
| `nbody.c` | Gravity | Velocity Verlet, softening |
| `cloth.c` | Spring-mass | Hooke's law, symplectic Euler |
| `elastic_collision.c` | Billiards | momentum + KE conservation |
| `orbit_3body.c` | Three-body | Verlet, figure-8 IC |
| `gyroscope.c` | Rigid body | Euler angles, torque |
| `magnetic_field.c` | Electromagnetism | Biot-Savart superposition, RK4 streamline tracing, aspect-corrected vector field |
| `chain.c` | Position-Based Dynamics | Verlet prediction, iterative distance-constraint projection, tension coloring, wave propagation |
| `rigid_body.c` | 2D rigid body collisions — unified AABB | Single `col_bodies()` for all pairs; sphere AABB `hw=r, hh=2r` (terminal aspect fix); two-pass: Baumgarte always + velocity impulse when approaching; adaptive `e_eff`; Coulomb friction; spawn overlap check; sleep counter |
| `soft_body.c` | Jelly blob — spring-mass mesh + pressure | Hooke springs (structural/shear/bending), shoelace area, pressure force on boundary edges, symplectic Euler |
| `beam_bending.c` | Euler-Bernoulli beam statics + modal dynamics | Analytical w(x)/M(x) for 9 BC×load combos; eigenmode superposition; exact damped oscillator step (unconditionally stable) |
| `diff_drive_robot.c` | Differential drive robot kinematics | v=(vL+vR)/2, ω=(vR-vL)/L; nonholonomic constraint; pixel-space Euler integration; ICC turn radius |

---

## Tier 2.5 — Character Animation (2–3 months)
**Prerequisites: Tier 2 (Verlet integration, spring-mass, constraint solvers).**
Bridges kinematic geometry with the physics patterns from Tier 2. Introduces articulated bodies, procedural locomotion, and constraint-based simulation at the character scale.

| File | Subject | Key Math |
|---|---|---|
| `snake_forward_kinematics.c` | Circular trail-buffer FK, bead rendering, 10 themes | Sinusoidal heading, ring-buffer position chain |
| `snake_inverse_kinematics.c` | IK head steering with FK body | Multi-harmonic wandering target, trail-buffer FK |
| `fk_centipede.c` | Trail-buffer FK body + stateless sinusoidal FK legs | Contralateral antiphase gait, phase-offset per leg pair |
| `fk_tentacle_forest.c` | Pure stateless sinusoidal FK, multi-tentacle scene | Per-tentacle phase/frequency/amplitude parameters |
| `fk_medusa.c` | Bell oscillation FK + trailing tentacle FK | Radial bell deformation, cascaded FK chains |
| `ik_arm_reach.c` | FABRIK iterative IK (2-pass solver) | Forward/backward reaching passes, Lissajous target path |
| `ik_spider.c` | 2-joint analytical IK, procedural stepping, arrow-key steering | Law of cosines, step trigger + lerp, trail-buffer body, short-arc heading interpolation |
| `hexpod_tripod.c` | 6-legged robot: timer-based tripod gait + 2-joint IK, 4-dir steering | Law of cosines IK, parabolic foot arc, angular interpolation, rotate2d body-local → world |
| `ik_tentacle_seek.c` | FABRIK on seeking tentacles | Wandering target, per-segment reach tolerance |
| `ragdoll_figure.c` | Verlet ragdoll skeleton | Verlet integration, iterative distance-constraint projection |
| `ragdoll_ropes.c` | Verlet rope chains, multi-rope sway | Damping, phase-offset anchors, constraint relaxation |
| `walking_robot.c` | Procedural bipedal walk cycle — sinusoidal FK, 2-joint analytical IK stance, foot contact locking | Sinusoidal FK gait, law of cosines IK, COM projection, shadow ellipse, motion trails |

---

## Tier 2.6 — Control Systems (1–2 months)
**Prerequisites: Tier 2 (Lagrangian mechanics, Euler integration) and Tier 2.5 (IK, procedural locomotion).**
Introduces closed-loop feedback control, stability analysis, and the phase-space view of dynamical systems.

| File | Subject | Key Math |
|---|---|---|
| `perlin_terrain_bot.c` | Self-balancing wheel bot on infinite Perlin terrain | Lagrangian cart-pole on slope: θ_ddot = (g·sin(θ_eff) − ẍ·cos(θ_eff))/L; PID controller (P/I/D terms), integral windup clamp; cascade slope feed-forward θ_ref = −α×0.65; 1D Perlin fBm terrain ring buffer; phase portrait (θ vs ω); six named gain presets (BALANCED/HIGH Kp/LOW Kp/NO Kd/HIGH Kd/NO Ki); regime classification (stable/settling/underdamped/overdamped) |

---

## Tier 3 — Fluid and Continuous Systems (3–4 months)
Requires partial differential equations and numerical stability.

| File | Subject | Key Math |
|---|---|---|
| `lattice_gas.c` | FHP-I lattice gas | 6-dir bit-packed hex, collision lookup table, streaming + bounce-back, Navier-Stokes emerges |
| `wave.c` | 1D/2D wave PDE | FDTD, CFL condition |
| `wave_2d.c` | 2D wave PDE | Huygens, interference |
| `nuke.c` | 2D shockwave | scalar wave PDE + cylindrical 1/√r decay + γ damping; CFL substepping; terrain ripple coupling; debris/dust particles; decaying screen shake |
| `reaction_diffusion.c` | Gray-Scott | Laplacian, diffusion |
| `sand.c` | Cellular automaton | simple rules |
| `flowfield.c` | Vector field | fBm Perlin noise |
| `navier_stokes.c` | Stable fluid | Gauss-Seidel, projection, advection |
| `lenia.c` | Continuous GoL | convolution kernel, growth fn |
| `marching_squares.c` | Isosurface extraction | 16-case 4-bit lookup, linear edge interpolation, metaball scalar field |
| `fluid_sph.c` | SPH particle fluid | kernel density estimation (w=(d/H−1)²), pressure + viscosity forces, symplectic Euler, O(N·k) linked-list spatial grid |

---

## Tier 4 — Advanced Physics and Math (4–6 months)
Requires linear algebra, complex numbers, statistical mechanics.

| File | Subject | Key Math |
|---|---|---|
| `ising.c` | Stat mechanics | Metropolis MCMC, phase transition |
| `schrodinger.c` | Quantum mech | Crank-Nicolson, Thomas algorithm |
| `orbit_3body.c` | N-body chaos | figure-8 IC, Verlet |
| `mandelbrot.c` | Complex iteration | escape-time, complex plane |
| `julia.c` | Complex dynamics | same as mandelbrot, c fixed |
| `buddhabrot.c` | Orbit density | two-pass sampling, log-normalize |
| `newton_fractal.c` | Complex Newton | Newton-Raphson, basin coloring |
| `strange_attractor.c` | Chaos | Clifford/de Jong map, density accumulation |
| `apollonian.c` | Circle packing | Descartes theorem, recursion |
| `bifurcation.c` | Chaos theory | logistic map, period doubling |
| `burning_ship.c` | Complex fractal | modified Mandelbrot, abs() trick |

---

## Tier 5 — Rendering (parallel with Tier 3–4)
Requires linear algebra and graphics pipeline understanding.

| File | Subject | Key Math |
|---|---|---|
| `wireframe.c` | 3D projection | rotation matrix, perspective |
| `donut.c` | Parametric surface | torus parametrization |
| `raymarcher.c` | SDF ray marching | sphere marching, Blinn-Phong |
| `raymarcher_cube.c` | SDF box | finite-difference normals |
| `raymarcher_primitives.c` | SDF ops | min/max composition |
| `metaballs.c` | Implicit surfaces | summed SDFs |
| `torus_raster.c` | Rasterization | barycentric coords, z-buffer |
| `cube_raster.c` | Shader pipeline | vert/frag shaders, back-face cull |
| `sphere_raster.c` | UV mapping | spherical coordinates |
| `displace_raster.c` | Vertex displacement | central-difference normals |
| `penrose.c` | Aperiodic tiling | de Bruijn pentagrid duality |
| `sphere_raytrace.c` | Analytic ray tracing | quadratic ray-sphere, 3-point Phong, Fresnel, 256-color |
| `cube_raytrace.c` | Analytic ray tracing | AABB slab method, inverse-rotation transform, face normals, wireframe |
| `torus_raytrace.c` | Analytic ray tracing | quartic via sampling+bisection, gradient normal, Horner evaluation |
| `capsule_raytrace.c` | Analytic ray tracing | cylinder+sphere-cap decomposition, axial projection normal, inverse-rotation transform |
| `blackhole.c` | GR null-geodesic raytracer | Schwarzschild metric (RK4), precomputed lensing table, photon ring, Doppler beaming D=[(1+β)/(1−β)]^1.5, gravitational redshift, 11 themes |
| `mandelbulb_explorer.c` | 3D fractal raymarcher | spherical power DE, smooth coloring, orbit traps, AO, soft shadows, progressive rendering, 8 multi-hue themes |
| `mandelbulb_raster.c` | Fractal rasterization | sphere projection tessellation, HSV fragment shaders, rgb_to_cell 216-color, frag_phong_hue / frag_normals / frag_depth_hue |
| `sdf_gallery.c` | SDF composition gallery | boolean ops min/max, smin polynomial blend, twist pre-warp, domain repetition, 5 scenes, 3 lighting modes (N·V/Phong/Flat), gamma-encoded Bourke ramp, bright hue-varying themes |
| `path_tracer.c` | Monte Carlo path tracing | Lambertian BRDF, cosine hemisphere sampling (Malley), Russian roulette, progressive accumulator, Reinhard tone map, Cornell Box, xorshift32 RNG |
| `sun.c` | Animated solar SDF + flares | sphere-trace SDF, value-noise + warped fBm displacement, capsule flare smin-blend, `bezier_tube_dist()` Bézier-arched magnetic loops, exponential corona accumulator, Eddington 1-coefficient limb darkening |
| `nuke_v1.c` | Volumetric mushroom cloud | Beer–Lambert front-to-back integrator (no SDFs), single morphing anisotropic Gaussian blob, quintic smootherstep continuous-time morph over overlapping windows, domain-warped fBm displacement, 2× vertical supersampling + sub-cell glyph picker, ash terminal-velocity drag, plateau→fall phase fade |

---

## Tier 6 — Emergent Systems (combine with Tier 5)
Agent-based simulation, graph algorithms, complex systems.

| File | Subject | Key Math |
|---|---|---|
| `flocking.c` | Reynolds boids | separation+alignment+cohesion |
| `shepherd.c` | Herding | flee force, bounded domain |
| `ant_colony.c` | ACO | pheromone decay, stigmergy |
| `voronoi.c` | Voronoi diagram | nearest-neighbor, Langevin |
| `wator.c` | Predator-prey | Wa-Tor rules |
| `graph_search.c` | BFS/DFS/A* | graph traversal |
| `network_sim.c` | Network dynamics | node/edge simulation |
| `reaction_wave.c` | Excitable media | FitzHugh-Nagumo |
| `maze.c` | Graph algorithms | DFS generation, BFS solve |
| `convex_hull.c` | Computational geom | Graham scan, Jarvis march |
| `quad_tree_helloworld.c` | Quadtree (animated ncurses) | PR quadtree, AABB pruning, static pool, depth-coloured INSERT→QUERY phases |
| `quadtree.c` | Quadtree (pure C demo) | half-open intervals, recursive subdivision, range query O(log N + k) |
| `bsp_tree.c` | BSP tree (pure C demo) | alternating axis splits, front/back children, Doom/Quake lineage |
| `sort_vis.c` | Algorithms | bubble/insertion/selection/quick/heap |
| `forest_fire.c` | Drossel-Schwabl CA | 3-state probabilistic update, neighbour spread + lightning, SOC power-law fire sizes |
| `slime_mold.c` | Physarum agent model | Jeff Jones (2010) sense-rotate-move-deposit, trail diffusion + decay, emergent Steiner networks |
| `crowd.c` | Reynolds steering crowd | 6 behaviours (WANDER/FLOCK/PANIC/GATHER/FOLLOW/QUEUE); seek/flee/separate/align/cohesion; momentum-based steering; `force = desired_vel − current_vel` |
| `war.c` | Battle simulation FSM | 4-state FSM (ADVANCE/COMBAT/FLEE/DEAD); travelling projectile arrows; flat pool; 6 live StrategyParams presets via g_sp pointer |
| `swarm_gen_numbers.c` | Swarm digit formation | 10 strategies; greedy slot assignment O(N×S); steer_arrive deceleration; steer_spring Hooke's law; SEP_BASE_FORCE independence; force balance design |

---

## Terrain and Noise

| File | Subject | Key Math |
|---|---|---|
| `terrain.c` | Diamond-square | fractal subdivision, erosion |
| `perlin_landscape.c` | fBm terrain | Perlin noise, parallax |
| `plasma.c` | Plasma effect | sin-sum palette cycling |

---

## 2-Year Breakdown

| Period | Focus | Milestone |
|---|---|---|
| Month 1–2 | Framework + Tier 1 | Rewrite 5 Tier 1 files from scratch |
| Month 3–5 | Tier 2 | Understand RK4, Verlet; rewrite lorenz from paper |
| Month 6–8 | Tier 2.5 — Character Animation | Implement FK chain from scratch; solve a 2-joint IK analytically; build a Verlet ragdoll |
| Month 8–9 | Tier 2.6 — Control Systems | Derive Lagrangian cart-pole EOM; implement PID from scratch; tune gains using phase portrait |
| Month 9–12 | Tier 3 | Implement Gray-Scott and Navier-Stokes from equations alone |
| Month 13–17 | Tier 4 | Derive Crank-Nicolson; implement Ising from Metropolis paper |
| Month 18–21 | Tier 5 | Build a software rasterizer from scratch |
| Month 22–24 | Combine + Create | Your own topic, your own algorithm |

---

## Books Alongside Each Tier

| Tier | Book |
|---|---|
| 1 | *The Nature of Code* — Daniel Shiffman |
| 2 | *Physics-Based Animation* — Kenny Erleben |
| 2.5 | *Computer Animation: Algorithms and Techniques* — Rick Parent |
| 2.6 | *Feedback Control of Dynamic Systems* — Franklin, Powell & Emami-Naeini |
| 3 | *Fluid Simulation for Computer Graphics* — Robert Bridson |
| 4 | *Statistical Mechanics* — Reif; *Quantum Mechanics* — Griffiths |
| 5 | *Real-Time Rendering* — Akenine-Möller |
| 6 | *Complex Adaptive Systems* — Miller & Page |
