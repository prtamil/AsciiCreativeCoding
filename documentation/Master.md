# Master — Concepts, Techniques & Mental Models

Everything used across this project, explained from first principles.
Use this as a reading map: scan the index, pick what you do not know, read the essay.

---

## Index

### A — Terminal & ncurses
- [A1 ncurses Initialization & Session Lifecycle](#a1-ncurses-initialization--session-lifecycle)
- [A2 Internal Double Buffer — newscr / curscr / doupdate](#a2-internal-double-buffer--newscr--curscr--doupdate)
- [A3 erase() vs clear()](#a3-erase-vs-clear)
- [A4 Color Pairs & Attributes](#a4-color-pairs--attributes)
- [A5 256-color vs 8-color Fallback](#a5-256-color-vs-8-color-fallback)
- [A6 typeahead(-1) — Preventing Mid-flush Input Poll](#a6-typeahead-1--preventing-mid-flush-input-poll)
- [A7 nodelay & Non-blocking Input](#a7-nodelay--non-blocking-input)
- [A8 SIGWINCH — Terminal Resize](#a8-sigwinch--terminal-resize)
- [A9 use_default_colors — Transparent Terminal Background](#a9-use_default_colors--transparent-terminal-background)
- [A10 ACS Line-drawing Characters](#a10-acs-line-drawing-characters)
- [A11 (chtype)(unsigned char) — Sign-extension Guard](#a11-chtypeunsigned-char--sign-extension-guard)
- [A12 attr_t — Composing Color + Attribute in One Variable](#a12-attr_t--composing-color--attribute-in-one-variable)
- [A13 Dynamic Color — Re-registering Pairs Mid-animation](#a13-dynamic-color--re-registering-pairs-mid-animation)

### B — Timing & Loop Architecture
- [B1 Monotonic Clock — clock_gettime(CLOCK_MONOTONIC)](#b1-monotonic-clock--clock_gettimeclock_monotonic)
- [B2 Fixed-timestep Accumulator](#b2-fixed-timestep-accumulator)
- [B3 dt Cap — Spiral-of-death Prevention](#b3-dt-cap--spiral-of-death-prevention)
- [B4 Frame Cap — Sleep Before Render](#b4-frame-cap--sleep-before-render)
- [B5 Render Interpolation — Alpha](#b5-render-interpolation--alpha)
- [B6 Forward Extrapolation vs Lerp](#b6-forward-extrapolation-vs-lerp)
- [B7 FPS Counter — Rolling Average](#b7-fps-counter--rolling-average)

### C — Coordinate Systems & Aspect Ratio
- [C1 Pixel Space vs Cell Space](#c1-pixel-space-vs-cell-space)
- [C2 px_to_cell — Round-half-up vs roundf](#c2-px_to_cell--round-half-up-vs-roundf)
- [C3 Aspect Ratio in Projection Matrices](#c3-aspect-ratio-in-projection-matrices)
- [C4 Ray Direction Aspect Correction (Raymarching)](#c4-ray-direction-aspect-correction-raymarching)

### D — Physics Simulation
- [D1 Euler Integration](#d1-euler-integration)
- [D2 Semi-implicit (Symplectic) Euler](#d2-semi-implicit-symplectic-euler)
- [D3 Wall Bounce — Elastic Reflection](#d3-wall-bounce--elastic-reflection)
- [D4 Gravity & Drag (Particle Systems)](#d4-gravity--drag-particle-systems)
- [D5 Spring-Pendulum — Lagrangian Mechanics](#d5-spring-pendulum--lagrangian-mechanics)
- [D6 Lifetime & Exponential Decay](#d6-lifetime--exponential-decay)
- [D7 Particle Pool — Fixed Array, No Allocation](#d7-particle-pool--fixed-array-no-allocation)
- [D8 State Machines in Physics Objects](#d8-state-machines-in-physics-objects)
- [D9 Impulse-based Elastic Collision Resolution](#d9-impulse-based-elastic-collision-resolution)
- [D10 Lorentz Force — Exact Rotation Integration](#d10-lorentz-force--exact-rotation-integration)
- [D11 Euler Equations for Rigid-Body Rotation — Quaternion Tracking](#d11-euler-equations-for-rigid-body-rotation--quaternion-tracking)
- [D12 Three-Body Choreography — Figure-8 Orbit](#d12-three-body-choreography--figure-8-orbit)
- [D13 Pendulum Wave — Analytic Harmonic Superposition](#d13-pendulum-wave--analytic-harmonic-superposition)
- [D14 Slider-Crank Kinematics — Engine Cycle Simulation](#d14-slider-crank-kinematics--engine-cycle-simulation)
- [D15 Baumgarte Stabilised Rigid-Body Collision](#d15-baumgarte-stabilised-rigid-body-collision)
- [D16 Quantum Wavepacket — Crank-Nicolson / Thomas Algorithm](#d16-quantum-wavepacket--crank-nicolson--thomas-algorithm)
- [D17 Euler-Bernoulli Beam — Analytical Statics + Modal Dynamics](#d17-euler-bernoulli-beam--analytical-statics--modal-dynamics)
- [D18 Differential Drive Robot — Nonholonomic Kinematics](#d18-differential-drive-robot--nonholonomic-kinematics)
- [D19 Inverted Pendulum — Lagrangian Cart-Pole on Slope](#d19-inverted-pendulum--lagrangian-cart-pole-on-slope)
- [D20 PID Controller — Proportional/Integral/Derivative with Windup Clamp](#d20-pid-controller--proportionalintegralderivative-with-windup-clamp)
- [D21 Vorticity-Streamfunction (VSF) Navier-Stokes — fluid/vorticity_streamfunction_solver.c](#d21-vorticity-streamfunction-vsf-navier-stokes)
- [D22 Acoustic FDTD — Leapfrog Scalar Wave Equation](#d22-acoustic-fdtd--leapfrog-scalar-wave-equation)
- [D23 Lattice Boltzmann Method (D2Q9) — LBM Fluid](#d23-lattice-boltzmann-method-d2q9--lbm-fluid)
- [D24 ODE Integrator Zoo — Euler / RK2 / RK4 / Velocity Verlet](#d24-ode-integrator-zoo--euler--rk2--rk4--velocity-verlet)
- [D25 Short-Time Fourier Transform — STFT Spectrogram](#d25-short-time-fourier-transform--stft-spectrogram)
- [D26 Deferred Rendering Pipeline — G-buffer + Blinn-Phong Lighting Pass](#d26-deferred-rendering-pipeline--g-buffer--blinn-phong-lighting-pass)

### E — Cellular Automata & Grid Simulations
- [E1 Falling Sand — Gravity CA](#e1-falling-sand--gravity-ca)
- [E2 Doom-style Fire — Heat Diffusion CA](#e2-doom-style-fire--heat-diffusion-ca)
- [E3 aafire 5-Neighbour CA](#e3-aafire-5-neighbour-ca)
- [E4 Processing Order & Artefact Suppression](#e4-processing-order--artefact-suppression)
- [E5 Stochastic Rules](#e5-stochastic-rules)
- [E6 Hexagonal Grid CA — Offset-Row Neighbour Addressing](#e6-hexagonal-grid-ca--offset-row-neighbour-addressing)
- [E7 Lenia — Continuous CA with Ring Kernel](#e7-lenia--continuous-ca-with-ring-kernel)
- [E8 Greenberg-Hastings Excitable Media CA](#e8-greenberg-hastings-excitable-media-ca)
- [E9 Marching Squares — Scalar Field Contour Extraction](#e9-marching-squares--scalar-field-contour-extraction)

### F — Noise & Procedural Generation
- [F1 Perlin Noise — Permutation Table & Smoothstep](#f1-perlin-noise--permutation-table--smoothstep)
- [F2 Octave Layering (Fractal Brownian Motion)](#f2-octave-layering-fractal-brownian-motion)
- [F3 Flow Field from Noise](#f3-flow-field-from-noise)
- [F4 Curl Noise — Divergence-Free Vector Fields](#f4-curl-noise--divergence-free-vector-fields)
- [F5 Biot-Savart Vortex Lattice](#f5-biot-savart-vortex-lattice)
- [F6 Pre-Baked Cosine Palette → 16 Color Pairs](#f6-pre-baked-cosine-palette--16-color-pairs)
- [F4 LCG — Deterministic Pseudo-random Numbers](#f4-lcg--deterministic-pseudo-random-numbers)
- [F5 Rejection Sampling — Isotropic Random Direction](#f5-rejection-sampling--isotropic-random-direction)

### G — ASCII Rendering & Dithering
- [G1 Paul Bourke ASCII Density Ramp](#g1-paul-bourke-ascii-density-ramp)
- [G2 Bayer 4×4 Ordered Dithering](#g2-bayer-44-ordered-dithering)
- [G3 Floyd-Steinberg Error Diffusion Dithering](#g3-floyd-steinberg-error-diffusion-dithering)
- [G4 Luminance — Perceptual RGB Weighting](#g4-luminance--perceptual-rgb-weighting)
- [G5 Gamma Correction](#g5-gamma-correction)
- [G6 Directional Characters — Arrow & Line Glyphs](#g6-directional-characters--arrow--line-glyphs)
- [G7 Stippled Lines — Distance-fade Bresenham](#g7-stippled-lines--distance-fade-bresenham)
- [G8 cell_used Grid — Line Deduplication](#g8-cell_used-grid--line-deduplication)
- [G9 Scorch Mark Persistence](#g9-scorch-mark-persistence)
- [G10 Bright Hue-varying Theme Palette](#g10-bright-hue-varying-theme-palette)

### H — 3D Math
- [H1 Vec3 / Vec4 — Inline Struct Math](#h1-vec3--vec4--inline-struct-math)
- [H2 Mat4 — 4×4 Homogeneous Matrix](#h2-mat4--44-homogeneous-matrix)
- [H3 Model / View / Projection (MVP)](#h3-model--view--projection-mvp)
- [H4 Perspective Projection Matrix](#h4-perspective-projection-matrix)
- [H5 Look-at Matrix](#h5-look-at-matrix)
- [H6 Normal Matrix — Cofactor of Model 3×3](#h6-normal-matrix--cofactor-of-model-33)
- [H7 Rotation Matrices (X, Y axes)](#h7-rotation-matrices-x-y-axes)
- [H8 Perspective Divide — Clip to NDC](#h8-perspective-divide--clip-to-ndc)
- [H9 Cross Product & Dot Product](#h9-cross-product--dot-product)

### I — Raymarching & SDF
- [I1 Signed Distance Functions (SDF)](#i1-signed-distance-functions-sdf)
- [I2 Sphere Marching Loop](#i2-sphere-marching-loop)
- [I3 SDF Normal via Finite Difference](#i3-sdf-normal-via-finite-difference)
- [I4 SDF Primitives — Sphere, Box, Torus](#i4-sdf-primitives--sphere-box-torus)
- [I5 Mandelbulb Distance Estimator](#i5-mandelbulb-distance-estimator)
- [I6 Smooth Escape-time Coloring](#i6-smooth-escape-time-coloring)
- [I7 Orbit Trap Coloring](#i7-orbit-trap-coloring)
- [I8 Near-miss Glow via min_d Tracking](#i8-near-miss-glow-via-min_d-tracking)
- [I9 Progressive ROWS_PER_TICK Rendering](#i9-progressive-rows_per_tick-rendering)
- [I10 SDF Boolean Operations — Union, Intersection, Subtraction](#i10-sdf-boolean-operations--union-intersection-subtraction)
- [I11 Smooth Union — Polynomial smin Blend](#i11-smooth-union--polynomial-smin-blend)
- [I12 Twist Deformation — Pre-warp Before SDF Evaluation](#i12-twist-deformation--pre-warp-before-sdf-evaluation)
- [I13 Domain Repetition — Infinite Lattice from One Primitive](#i13-domain-repetition--infinite-lattice-from-one-primitive)
- [I14 Analytic Ray-Capsule Intersection](#i14-analytic-ray-capsule-intersection)
- [I15 Quartic Torus Ray Intersection — Sampling + Bisection](#i15-quartic-torus-ray-intersection--sampling--bisection)

### J — Software Rasterization
- [J1 Mesh — Vertex & Triangle Arrays](#j1-mesh--vertex--triangle-arrays)
- [J2 UV Sphere Tessellation](#j2-uv-sphere-tessellation)
- [J3 Torus Tessellation](#j3-torus-tessellation)
- [J4 Cube Tessellation — Flat Normals](#j4-cube-tessellation--flat-normals)
- [J5 Vertex Shader — VSIn / VSOut](#j5-vertex-shader--vsin--vsout)
- [J6 Fragment Shader — FSIn / FSOut](#j6-fragment-shader--fsin--fsout)
- [J7 ShaderProgram — Split vert_uni / frag_uni](#j7-shaderprogram--split-vert_uni--frag_uni)
- [J8 Barycentric Coordinates](#j8-barycentric-coordinates)
- [J9 Barycentric Interpolation of Vertex Attributes](#j9-barycentric-interpolation-of-vertex-attributes)
- [J10 Z-buffer (Depth Buffer)](#j10-z-buffer-depth-buffer)
- [J11 Back-face Culling — Screen-space Signed Area](#j11-back-face-culling--screen-space-signed-area)
- [J12 Near-plane Clip Reject](#j12-near-plane-clip-reject)
- [J13 Framebuffer — zbuf + cbuf](#j13-framebuffer--zbuf--cbuf)
- [J14 Barycentric Wireframe](#j14-barycentric-wireframe)
- [J15 Vertex Displacement](#j15-vertex-displacement)
- [J16 Central Difference Normal Recomputation](#j16-central-difference-normal-recomputation)
- [J17 Tangent Basis Construction](#j17-tangent-basis-construction)
- [J18 Sphere Projection Tessellation (Implicit Surface → Mesh)](#j18-sphere-projection-tessellation-implicit-surface--mesh)
- [J19 rgb_to_cell — Full-color Framebuffer via 216-color Cube](#j19-rgb_to_cell--full-color-framebuffer-via-216-color-cube)
- [J20 HSV → RGB for Fragment Shaders](#j20-hsv--rgb-for-fragment-shaders)

### K — Shading Models
- [K1 Blinn-Phong Shading](#k1-blinn-phong-shading)
- [K2 Toon / Cel Shading — Banded Diffuse](#k2-toon--cel-shading--banded-diffuse)
- [K3 Normal Visualisation Shader](#k3-normal-visualisation-shader)
- [K4 Parametric Torus Lighting (Donut)](#k4-parametric-torus-lighting-donut)
- [K5 3-Point Lighting Setup — Key + Fill + Rim](#k5-3-point-lighting-setup--key--fill--rim)
- [K6 Depth Visualization Shader — View-space Depth → Color](#k6-depth-visualization-shader--view-space-depth--color)
- [K7 Three Lighting Modes — N·V / Phong / Flat](#k7-three-lighting-modes--nv--phong--flat)

### L — Algorithms & Data Structures
- [L1 Bresenham Line Algorithm](#l1-bresenham-line-algorithm)
- [L2 Ring Buffer](#l2-ring-buffer)
- [L3 Z-buffer / Depth Sort](#l3-z-buffer--depth-sort)
- [L4 Bounding Box Rasterization](#l4-bounding-box-rasterization)
- [L5 Fisher-Yates Shuffle](#l5-fisher-yates-shuffle)
- [L6 Callback / Function Pointer Patterns](#l6-callback--function-pointer-patterns)
- [L7 Lookup Table (LUT)](#l7-lookup-table-lut)
- [L8 Bilinear Interpolation](#l8-bilinear-interpolation)
- [L9 Graham Scan & Jarvis March — Convex Hull](#l9-graham-scan--jarvis-march--convex-hull)
- [L10 BFS / DFS / A* Graph Search](#l10-bfs--dfs--a-graph-search)
- [L11 DFS Maze Generation — Recursive Backtracker](#l11-dfs-maze-generation--recursive-backtracker)
- [L12 Sorting Algorithm Step-Iterator — Coroutine Pattern](#l12-sorting-algorithm-step-iterator--coroutine-pattern)
- [L13 Hexagonal Grid Coordinate Systems](#l13-hexagonal-grid-coordinate-systems)

### M — Systems Programming
- [M1 Signal Handling — sig_atomic_t](#m1-signal-handling--sig_atomic_t)
- [M2 atexit — Guaranteed Cleanup](#m2-atexit--guaranteed-cleanup)
- [M3 nanosleep — High-resolution Sleep](#m3-nanosleep--high-resolution-sleep)
- [M4 Static Global App — Signal Handler Reach](#m4-static-global-app--signal-handler-reach)
- [M5 void* Uniform Casting — Type-safe Shader Interface](#m5-void-uniform-casting--type-safe-shader-interface)
- [M6 memset for Zero-init](#m6-memset-for-zero-init)
- [M7 size_t Casts in malloc](#m7-size_t-casts-in-malloc)

### N — Flocking & Collective Behaviour
- [N1 Classic Boids — Separation / Alignment / Cohesion](#n1-classic-boids--separation--alignment--cohesion)
- [N2 Vicsek Model — Noise-driven Phase Transition](#n2-vicsek-model--noise-driven-phase-transition)
- [N3 Leader Chase](#n3-leader-chase)
- [N4 Orbit Formation](#n4-orbit-formation)
- [N5 Predator-Prey](#n5-predator-prey)
- [N6 Toroidal Topology — Wrap-around Physics](#n6-toroidal-topology--wrap-around-physics)
- [N7 Shepherd / Herding — Flee Force with Panic Boost](#n7-shepherd--herding--flee-force-with-panic-boost)
- [N8 Ant Colony Optimization — Stigmergic Pheromone Trails](#n8-ant-colony-optimization--stigmergic-pheromone-trails)

### O — Procedural Growth
- [O1 Recursive Branch Growth — Bonsai](#o1-recursive-branch-growth--bonsai)
- [O2 Branch Type State Machine](#o2-branch-type-state-machine)
- [O3 Leaf Scatter](#o3-leaf-scatter)

### P — Fractal Systems
- [P1 Diffusion-Limited Aggregation (DLA)](#p1-diffusion-limited-aggregation-dla)
- [P2 D6 Hexagonal Symmetry with Aspect Correction](#p2-d6-hexagonal-symmetry-with-aspect-correction)
- [P3 Anisotropic DLA — Directional Sticking](#p3-anisotropic-dla--directional-sticking)
- [P4 Chaos Game / Iterated Function System (IFS)](#p4-chaos-game--iterated-function-system-ifs)
- [P5 Barnsley Fern — 4-Transform IFS](#p5-barnsley-fern--4-transform-ifs)
- [P6 Escape-time Iteration — Julia and Mandelbrot Sets](#p6-escape-time-iteration--julia-and-mandelbrot-sets)
- [P7 Fisher-Yates Random Pixel Reveal](#p7-fisher-yates-random-pixel-reveal)
- [P8 Koch Snowflake — Midpoint Subdivision](#p8-koch-snowflake--midpoint-subdivision)
- [P9 Recursive Tip Branching — Lightning](#p9-recursive-tip-branching--lightning)
- [P10 Buddhabrot — Density Accumulator Fractal](#p10-buddhabrot--density-accumulator-fractal)
- [P11 Pascal-Triangle Bat Swarms — Artistic Formation Flying](#p11-pascal-triangle-bat-swarms--artistic-formation-flying)
- [P12 De Bruijn Pentagrid — Penrose Tiling](#p12-de-bruijn-pentagrid--penrose-tiling)
- [P13 Diamond-Square Heightmap + Thermal Erosion](#p13-diamond-square-heightmap--thermal-erosion)
- [P16 Apollonian Gasket — Descartes Circle Theorem](#p16-apollonian-gasket--descartes-circle-theorem)
- [P17 Logistic Map Bifurcation Diagram — Feigenbaum Scaling](#p17-logistic-map-bifurcation-diagram--feigenbaum-scaling)
- [P18 Burning Ship Fractal — Absolute-Value Fold](#p18-burning-ship-fractal--absolute-value-fold)
- [P19 Dragon Curve — Paper-Folding Sequence](#p19-dragon-curve--paper-folding-sequence)
- [P20 Newton Fractal — Basin of Attraction Coloring](#p20-newton-fractal--basin-of-attraction-coloring)
- [P21 Strange Attractors — Density-Map Rendering](#p21-strange-attractors--density-map-rendering)
- [P22 Perlin Noise Landscape — Parallax Terrain Scrolling](#p22-perlin-noise-landscape--parallax-terrain-scrolling)
- [P23 Involute Gear Tooth Geometry](#p23-involute-gear-tooth-geometry)

### Q — Artistic / Signal-Based Effects
- [Q1 Sinusoidal Aurora Curtains](#q1-sinusoidal-aurora-curtains)
- [Q2 Demoscene Plasma — Sin-Sum Palette Cycling](#q2-demoscene-plasma--sin-sum-palette-cycling)
- [Q3 Hypotrochoid Spirograph with Float Canvas Decay](#q3-hypotrochoid-spirograph-with-float-canvas-decay)
- [Q4 Voronoi with Langevin Brownian Seeds](#q4-voronoi-with-langevin-brownian-seeds)
- [Q5 Cooley-Tukey Radix-2 FFT Visualizer](#q5-cooley-tukey-radix-2-fft-visualizer)
- [Q6 DFT Epicycles — Fourier Drawing Replay](#q6-dft-epicycles--fourier-drawing-replay)

### R — Force-Based Physics
- [R1 Explicit Spring Forces + Symplectic Euler (Cloth)](#r1-explicit-spring-forces--symplectic-euler-cloth)
- [R2 Softened N-Body Gravity + Verlet](#r2-softened-n-body-gravity--verlet)
- [R3 Lorenz Strange Attractor — RK4 + 3-D Projection](#r3-lorenz-strange-attractor--rk4--3-d-projection)

### S — Barnes-Hut & Galaxy Simulation
- [S1 Barnes-Hut Quadtree — O(N log N) Gravity](#s1-barnes-hut-quadtree--on-log-n-gravity)
- [S2 Incremental Centre-of-Mass Update](#s2-incremental-centre-of-mass-update)
- [S3 Static Node Pool — No malloc](#s3-static-node-pool--no-malloc)
- [S4 Keplerian Orbit Initialization](#s4-keplerian-orbit-initialization)
- [S5 Brightness Accumulator + Glow Decay](#s5-brightness-accumulator--glow-decay)
- [S6 Speed-Normalized Color Mapping](#s6-speed-normalized-color-mapping)

### T — Wave Physics & Signal Analysis
- [T1 Analytic Wave Interference — Phase Precomputation](#t1-analytic-wave-interference--phase-precomputation)
- [T2 Lyapunov Exponent — Alternating Logistic Map](#t2-lyapunov-exponent--alternating-logistic-map)
- [T3 DBM Laplace Growth — Gauss-Seidel + φ^η](#t3-dbm-laplace-growth--gauss-seidel--φη)
- [T4 2D Wave Equation — FDTD with Sponge Boundaries](#t4-2d-wave-equation--fdtd-with-sponge-boundaries)
- [T5 Navier-Stokes Stable Fluids — Stam's Method](#t5-navier-stokes-stable-fluids--stams-method)
- [T6 FitzHugh-Nagumo Reaction-Diffusion — Excitable Media](#t6-fitzhugh-nagumo-reaction-diffusion--excitable-media)

### U — Path Tracing & Global Illumination
- [U1 Monte Carlo Path Tracing](#u1-monte-carlo-path-tracing)
- [U2 Lambertian BRDF + Cosine Hemisphere Sampling](#u2-lambertian-brdf--cosine-hemisphere-sampling)
- [U3 Russian Roulette — Unbiased Path Termination](#u3-russian-roulette--unbiased-path-termination)
- [U4 Progressive Accumulator — Convergent Rendering](#u4-progressive-accumulator--convergent-rendering)
- [U5 Reinhard Tone Mapping](#u5-reinhard-tone-mapping)
- [U6 xorshift32 — Decorrelated Per-pixel RNG](#u6-xorshift32--decorrelated-per-pixel-rng)
- [U7 Cornell Box — Standard Path Tracing Scene](#u7-cornell-box--standard-path-tracing-scene)
- [U8 Axis-aligned Quad Intersection](#u8-axis-aligned-quad-intersection)

---

## Essays

---

### A — Terminal & ncurses

#### A1 ncurses Initialization & Session Lifecycle
`initscr()` captures the terminal into raw mode and creates the internal screen buffers. Every ncurses program pairs it with `endwin()` at exit — without `endwin()` the terminal stays in raw mode and the shell becomes unusable. The call sequence `initscr → noecho → cbreak → curs_set(0) → nodelay → keypad → color_init` is the standard boot order seen in every file here.
*Files: all files*

#### A2 Internal Double Buffer — newscr / curscr / doupdate
ncurses maintains two virtual screens internally: `curscr` (what the terminal currently shows) and `newscr` (what you are building). `doupdate()` computes the diff and sends only the changed cells as escape codes — one atomic write per frame. You never need a manual front/back buffer; adding one breaks the diff engine and produces ghost trails.
*Files: all files — explicitly used in `bounce.c`, `matrix_rain.c`, all raster files*

#### A3 erase() vs clear()
`erase()` wipes ncurses' internal `newscr` buffer with no terminal I/O; the diff engine will still send only what changed. `clear()` schedules a full `\e[2J` escape — every cell is retransmitted regardless of whether it changed, causing a full repaint every frame and visible flicker. Always use `erase()` in the render loop.
*Files: all files*

#### A4 Color Pairs & Attributes
ncurses cannot set foreground color alone — it uses `init_pair(n, fg, bg)` to register a numbered pair, then `COLOR_PAIR(n)` to apply it. `A_BOLD` brightens the foreground on both 8-color and 256-color terminals. All brightness gradients in this project are encoded as a sequence of color pairs rather than direct RGB.
*Files: all files*

#### A5 256-color vs 8-color Fallback
`COLORS >= 256` detects xterm-256 support at runtime. The 256-color path uses specific xterm palette indices (e.g., 196=bright red, 51=cyan) for rich gradients. The 8-color fallback uses `COLOR_RED`, `COLOR_GREEN` etc. with `A_BOLD`/`A_DIM` to approximate the same gradient. Both paths are gated by the same `if(COLORS>=256)` check in `color_init()`.
*Files: all files*

#### A6 typeahead(-1) — Preventing Mid-flush Input Poll
By default ncurses interrupts `doupdate()`'s output mid-stream to poll stdin for pending keystrokes. On fast terminals this breaks the atomic write into fragments, producing visible tearing. `typeahead(-1)` disables the poll entirely — ncurses writes the full diff without interruption.
*Files: all files*

#### A7 nodelay & Non-blocking Input
`nodelay(stdscr, TRUE)` makes `getch()` return `ERR` immediately when no key is available instead of blocking. Without it the main loop stalls waiting for input and the animation freezes. The pattern `int ch = getch(); if(ch != ERR) handle(ch);` appears identically in every file.
*Files: all files*

#### A8 SIGWINCH — Terminal Resize
The OS sends `SIGWINCH` when the terminal window is resized. The handler sets a `volatile sig_atomic_t need_resize = 1` flag; the main loop checks it at the top of each iteration. The actual resize (`endwin() → refresh() → getmaxyx()`) happens in the main loop, not the handler, because ncurses functions are not async-signal-safe.
*Files: all files*

---

### B — Timing & Loop Architecture

#### B1 Monotonic Clock — clock_gettime(CLOCK_MONOTONIC)
`CLOCK_MONOTONIC` is a hardware counter that never jumps backward and is unaffected by NTP or user clock changes. `CLOCK_REALTIME` can leap forward or backward mid-animation, producing a massive `dt` spike that causes physics to explode. Every file uses `clock_gettime(CLOCK_MONOTONIC, &t)` and converts to nanoseconds as `tv_sec * 1e9 + tv_nsec`.
*Files: all files*

#### B2 Fixed-timestep Accumulator
Wall-clock `dt` is added to a nanosecond bucket each frame; physics is stepped in fixed-size chunks until the bucket is exhausted. This decouples simulation accuracy from render frame rate — the physics always integrates at exactly `SIM_FPS` steps per second regardless of how long rendering takes, making it deterministic and numerically stable.
*Files: `bounce.c`, `matrix_rain.c`, `spring_pendulum.c`*

#### B3 dt Cap — Spiral-of-death Prevention
If the process was paused (debugger, OS suspend) and then resumed, the measured `dt` would be enormous and the accumulator would drain thousands of ticks in one frame. Clamping `dt` to 100 ms means the simulation appears to pause rather than fast-forward; the cap value is chosen to be imperceptible as lag but large enough to absorb any reasonable stall.
*Files: all files with accumulator*

#### B4 Frame Cap — Sleep Before Render
The sleep that limits output to 60 fps must happen *before* the terminal I/O (`doupdate`, `getch`), not after. If you sleep after, the measurement includes unpredictable terminal write time and the cap becomes erratic. By sleeping first — measuring only physics time — the cap is stable regardless of terminal speed.
*Files: all files — key insight documented in `Architecture.md`*

#### B5 Render Interpolation — Alpha
After draining the accumulator, `sim_accum` holds the leftover nanoseconds into the next unfired tick. `alpha = sim_accum / tick_ns` ∈ [0,1). Drawing objects at `pos + vel × alpha × dt` instead of at `pos` removes the 0–16 ms lag between physics state and wall-clock "now", eliminating micro-stutter.
*Files: `bounce.c`, `matrix_rain.c`, `spring_pendulum.c`*

#### B6 Forward Extrapolation vs Lerp
For constant-velocity motion (bouncing balls, falling characters), extrapolating forward by `alpha` is numerically identical to lerping between `prev` and `current` — and requires no extra storage. For non-linear forces (spring, pendulum), extrapolation diverges; the correct approach is storing `prev_r`, `prev_theta` and lerping: `draw = prev + (current - prev) × alpha`.
*Files: `bounce.c` (extrapolation), `spring_pendulum.c` (lerp)*

#### B7 FPS Counter — Rolling Average
Counting frames and accumulating time over a 500 ms window gives a display FPS that updates twice per second — stable enough to read without being laggy. Per-frame FPS (1/dt) oscillates too wildly to be useful; the rolling average smooths it.
*Files: all files*

---

### C — Coordinate Systems & Aspect Ratio

#### C1 Pixel Space vs Cell Space
Terminal cells are physically ~2× taller than wide (8 px wide × 16 px tall). Storing ball positions in cell coordinates and moving by `(1,1)` per tick travels twice as far horizontally in real pixels — circles become ellipses. The fix is to live in pixel space (`pos × CELL_W / CELL_H`) for physics and convert only at draw time.
*Files: `bounce.c`, `spring_pendulum.c`*

#### C2 px_to_cell — Round-half-up vs roundf
`floorf(px / CELL_W + 0.5f)` is "round half up" — always deterministic. `roundf` uses banker's rounding (round-half-to-even): when `px/CELL_W` is exactly 0.5, it may round to 0 on one call and 1 on the next depending on FPU state, causing a ball on a cell boundary to flicker between two cells every frame.
*Files: `bounce.c`, `matrix_rain.c`*

#### C3 Aspect Ratio in Projection Matrices
The perspective matrix receives `aspect = (cols × CELL_W) / (rows × CELL_H)` — the physical pixel aspect ratio, not just `cols/rows`. Without this, a rendered sphere appears as a vertical ellipse because terminal cells are taller than wide. All raster files use `CELL_W=8`, `CELL_H=16`.
*Files: all raster files*

#### C4 Ray Direction Aspect Correction (Raymarching)
In the raymarcher, the ray direction's Y component is divided by `CELL_ASPECT = 2.0` before normalization. Each terminal cell covers twice as many physical pixels vertically, so a ray stepping one cell down covers twice the screen distance of a ray stepping one cell right — the aspect divisor corrects this.
*Files: `raymarcher.c`, `raymarcher_cube.c`*

---

### D — Physics Simulation

#### D1 Euler Integration
The simplest integrator: `pos += vel × dt`, `vel += accel × dt`. It is first-order accurate and adds energy to oscillating systems over time (the orbit spirals outward). Used in particle systems and fire where energy drift doesn't matter because particles have finite lifetimes.
*Files: `fireworks.c`, `brust.c`, `kaboom.c`*

#### D2 Semi-implicit (Symplectic) Euler
Update velocity *before* position: `vel += accel × dt; pos += vel × dt`. This tiny reordering makes the integrator symplectic — it conserves a modified energy and does not spiral outward over time. Essential for oscillators like the spring-pendulum where standard Euler would visibly gain energy over seconds.
*Files: `spring_pendulum.c`*

#### D3 Wall Bounce — Elastic Reflection
When a ball crosses a boundary, clamp position to the boundary and negate the relevant velocity component. Doing it in the correct order (clamp then flip) prevents the ball from getting stuck inside the wall on the next tick. The raster files' `CAM_DIST_MIN/MAX` zoom clamp uses the same pattern.
*Files: `bounce.c`, `fireworks.c`*

#### D4 Gravity & Drag (Particle Systems)
Gravity adds a constant downward acceleration each tick (`vy += GRAVITY × dt`). Drag multiplies velocity by a factor less than 1 each tick (`vx *= 0.98`), simulating air resistance and preventing particles from flying off-screen forever. Exponential decay of `life` (`life *= DECAY`) drives the particle's visual fade.
*Files: `fireworks.c`, `brust.c`*

#### D5 Spring-Pendulum — Lagrangian Mechanics
The Lagrangian formulation derives equations of motion from kinetic minus potential energy, handling the coupling between spring extension and pendulum angle automatically. The result is two coupled second-order ODEs for `r̈` and `θ̈` that are integrated numerically each tick — more principled than writing forces by hand.
*Files: `spring_pendulum.c`*

#### D6 Lifetime & Exponential Decay
`life -= dt / lifetime_sec` counts down linearly; when it reaches 0 the particle is recycled. Multiplying by a `decay` factor less than 1 each tick gives exponential decay — the particle fades quickly at first then more slowly, matching the visual feel of embers cooling.
*Files: `fireworks.c`, `brust.c`, `matrix_rain.c` (trail fade)*

#### D7 Particle Pool — Fixed Array, No Allocation
All particle arrays are statically sized at init (`Particle pool[MAX]`). An `active` flag or lifetime <= 0 marks slots as free. Burst functions scan for inactive slots rather than calling `malloc`/`free` per particle — avoids heap fragmentation and allocation stalls in a 60 fps loop.
*Files: `fireworks.c`, `brust.c`, `kaboom.c`*

#### D8 State Machines in Physics Objects
Rockets cycle through `IDLE → RISING → EXPLODED`; fire columns have `COLD / HOT`; matrix columns have `ACTIVE / FADING`. A state machine makes transitions explicit and prevents illegal state combinations (e.g., exploding a rocket that hasn't launched). Each state drives a different code path in the tick function.
*Files: `fireworks.c`, `brust.c`, `matrix_rain.c`*

---

### E — Cellular Automata & Grid Simulations

#### E1 Falling Sand — Gravity CA
Each cell is either empty or sand. Each tick, process bottom-to-top: try to fall straight down; if blocked, try a random diagonal; if both blocked, try wind drift; otherwise mark stationary. Processing bottom-to-top prevents a grain from moving multiple cells in one tick (which would look like teleportation).
*Files: `sand.c`*

#### E2 Doom-style Fire — Heat Diffusion CA + Multi-Algorithm Grid Sims
The base CA rule: each cell samples ONE randomly-jittered neighbour from one row below and subtracts a decay term — not an average, a single sample. The lateral jitter (±1 col) is why flames flicker sideways rather than rising straight. The bottom row is arch-shaped fuel seeded every tick.

`fire.c` extends this with two additional algorithms switchable at runtime: **particle fire** (pool of embers 3×3 Gaussian-splatted onto the heat grid) and **plasma tongues** (sine-harmonic procedural columns, no persistent state). All three write into the same float grid and share the same Floyd-Steinberg + perceptual-LUT rendering pipeline. All tunable constants are named `#define` presets and shared helpers (`warmup_scale`, `advance_wind`, `arch_envelope`, `seed_fuel_row`, `splat3x3`) are factored out so no algo function duplicates the boilerplate.

`smoke.c` uses the same architecture with 3 smoke-specific algorithms: CA diffusion (±2 lateral jitter for broader billow), particle puffs (quadratic life² fade, bilinear splat), and vortex advection (Biot-Savart point vortices, semi-Lagrangian back-trace). Wind is accumulated once per tick in `scene_tick()` before dispatch to any algorithm.
*Files: `fire.c`, `smoke.c`*

#### E3 aafire 5-Neighbour CA
The aalib variant samples five neighbours (three below plus two diagonals two rows below) and averages them, producing rounder, slower-rising blobs compared to Doom's sharper spikes. A precomputed `minus` value based on screen height normalises the decay rate so the flame height is consistent at any terminal size.
*Files: `aafire_port.c`*

#### E4 Processing Order & Artefact Suppression
Scanning top-to-bottom in a falling CA lets grains move multiple cells in a single pass — they "teleport." Scanning bottom-to-top fixes this. For horizontal neighbours, randomising the left/right scan order each row removes the diagonal bias that otherwise makes all sand pile up on one side.
*Files: `sand.c`*

#### E5 Stochastic Rules
Adding `rand() % 2` to decide which diagonal direction to try, or whether to scatter a fire cell, gives organic variation with almost no code. Deterministic rules produce repetitive, crystalline patterns; a single random branch breaks the symmetry and makes the simulation look alive.
*Files: `sand.c`, `fire.c`, `aafire_port.c`, `flowfield.c`, `complex_flowfield.c`*

---

### F — Noise & Procedural Generation

#### F1 Perlin Noise — Permutation Table & Smoothstep
Ken Perlin's classic algorithm hashes integer grid corners using a 256-element permutation table, then blends the four corner contributions using a smoothstep curve (`6t⁵ - 15t⁴ + 10t³`). The result is a continuous, band-limited noise signal that looks natural — unlike `rand()` which has no spatial coherence.
*Files: `flowfield.c`, `complex_flowfield.c`*

#### F2 Octave Layering (Fractal Brownian Motion)
Summing multiple noise samples at increasing frequencies (`freq × 2ⁿ`) and decreasing amplitudes (`amp × 0.5ⁿ`) builds fractal detail. Two octaves give smooth hills; four give terrain with boulders; eight give bark texture. This project uses three octaves for the flow field angle, balancing detail against computation. `complex_flowfield.c` also uses multi-octave fBm for the curl noise scalar potential.
*Files: `flowfield.c`, `complex_flowfield.c`*

#### F3 Flow Field from Noise
Sample two independent noise fields at offset coordinates to get `(vx, vy)`, then `atan2(vy, vx)` gives an angle. Placing this angle at every grid cell builds a vector field that is spatially smooth but visually complex. Particles that follow the field produce curved, organic-looking trails.
*Files: `flowfield.c`, `complex_flowfield.c`*

#### F4 Curl Noise — Divergence-Free Vector Fields
Build a scalar potential ψ(x,y,t) from noise. The 2-D curl of a scalar field is a divergence-free vector field: `Vx = ∂ψ/∂y`, `Vy = −∂ψ/∂x`. Approximate with central differences: `Vx ≈ (ψ(x, y+ε) − ψ(x, y−ε)) / 2ε`. Divergence-free means `∂Vx/∂x + ∂Vy/∂y = 0` everywhere — no sources or sinks. Particles orbit indefinitely without clustering or dispersing, producing looping smoke-like motion that plain angle noise cannot achieve.
*Files: `complex_flowfield.c`*

#### F5 Biot-Savart Vortex Lattice
N point vortices each contribute a velocity at any grid cell via the 2-D Biot-Savart law: `Vx += S·(−dy)/(r²+ε)`, `Vy += S·dx/(r²+ε)`. Superimposing N vortices with alternating positive (CCW) and negative (CW) strengths creates complex whirlpool interference. The `ε` (VORT_EPS) term regularises the singularity at the vortex centre — without it, cells at exactly the vortex position receive infinite velocity. Placing vortices on a slowly rotating ring makes the pattern evolve continuously.
*Files: `complex_flowfield.c`, `particle_systems/smoke.c`*

#### F6 Pre-Baked Cosine Palette → 16 Color Pairs
The cosine palette formula `color(t) = a + b·cos(2π·(c·t+d))` generates smooth complementary hue gradients from just four 3-vectors per theme. Rather than calling `init_pair()` every frame (as `flocking.c` does), `complex_flowfield.c` calls it once at theme-change time for 16 evenly-spaced `t` values, registering all 16 color pairs upfront. Particles then select a pair by mapping their flow angle to `t ∈ [0,1]` — no per-frame color registration, no flicker. The `cos_to_xterm256()` function maps each float RGB triple to the xterm-256 color cube: `16 + 36·r5 + 6·g5 + b5` where each channel is rounded to the nearest of 6 discrete levels.
*Files: `complex_flowfield.c`*

#### F4 LCG — Deterministic Pseudo-random Numbers
A Linear Congruential Generator (`state = state × A + C mod 2³²`) produces a deterministic sequence from a seed. `kaboom.c` uses it so that the same seed always produces the same explosion shape — useful for pre-generating animation frames or making effects reproducible.
*Files: `kaboom.c`*

#### F5 Rejection Sampling — Isotropic Random Direction
Generating `(vx, vy)` as two independent uniform `[-1,1]` randoms and normalizing gives a non-uniform distribution — diagonal directions are more likely. The fix is to sample a point inside the unit circle by rejection: generate random `(x,y)` until `x² + y² <= 1`, then normalize. The result is a perfectly uniform angle distribution.
*Files: `bounce.c`*

---

### G — ASCII Rendering & Dithering

#### G1 Paul Bourke ASCII Density Ramp
A 92-character string ordered from visually sparse (space, backtick) to visually dense (`@`, `#`). Mapping a `[0,1]` luminance value to an index in this string converts brightness to an ASCII "pixel density" — dark regions get sparse characters, bright regions get dense ones. Used in every raster and raymarcher file.
*Files: all raster files, all raymarch files, `fire.c`*

#### G2 Bayer 4×4 Ordered Dithering
A precomputed 4×4 threshold matrix is added to each pixel's luminance before quantization: `dithered = luma + (bayer[y&3][x&3] - 0.5) × strength`. Ordered dithering introduces a regular, position-dependent pattern that encodes fractional brightness levels that the discrete character ramp cannot represent directly. It is fast (one table lookup per pixel) and produces clean halftone patterns.
*Files: all raster files, `fire.c`*

#### G3 Floyd-Steinberg Error Diffusion Dithering
After quantizing a pixel, the quantization error is distributed to the four unprocessed neighbours with weights `7/16, 3/16, 5/16, 1/16`. This "spends" the rounding error across adjacent pixels, producing smoother gradients than ordered dithering at the cost of a full-grid pass. Used in the fire renderers where smooth gradient quality is more important than pattern regularity.
*Files: `fire.c`, `aafire_port.c`*

#### G4 Luminance — Perceptual RGB Weighting
Human eyes are most sensitive to green and least to blue. Converting colour to brightness as `L = 0.2126R + 0.7152G + 0.0722B` (the ITU-R BT.709 coefficients) matches perceived brightness. Using a simple average `(R+G+B)/3` would make pure green look dim and pure blue look bright — the wrong result.
*Files: all raster files*

#### G5 Gamma Correction
Display hardware applies a nonlinear transfer function (gamma ≈ 2.2) to the stored color value. Working in linear light (as Phong shading does) and outputting without correction makes the image look too dark. Applying `pow(value, 1/2.2)` before output converts linear light to gamma-encoded display values and restores the correct perceptual brightness. In terminal renderers the same principle applies to Bourke char selection: without `pow(luma, 0.45)` before the ramp index calculation, linear luma values cluster in the top 2–3 chars and the lower density chars are rarely used, collapsing perceived contrast.
*Files: all raster files, `raymarcher.c`, `raymarcher/sdf_gallery.c`*

#### G6 Directional Characters — Arrow & Line Glyphs
In `flowfield.c` the particle head character is chosen by the angle of motion: `→ ↗ ↑ ↖ ← ↙ ↓ ↘`. Dividing `atan2(vy,vx)` by `π/4` and rounding to the nearest octant indexes into an 8-character array. `complex_flowfield.c` uses the same technique for both particle heads and the background colormap — every cell shows the flow direction glyph colored by the cosine palette. In `spring_pendulum.c` the spring is drawn with `/`, `\`, `|`, `-` chosen by the local segment slope.
*Files: `flowfield.c`, `complex_flowfield.c`, `spring_pendulum.c`, `wireframe.c`*

---

### H — 3D Math

#### H1 Vec3 / Vec4 — Inline Struct Math
All vector math uses plain C structs (`typedef struct { float x,y,z; } Vec3`) with inline helper functions. `static inline` lets the compiler eliminate the function call overhead entirely. The explicit field names (`v.x`, `v.y`, `v.z`) make the math readable; SIMD or arrays can be substituted later without changing the algorithm.
*Files: all raster and raymarcher files*

#### H2 Mat4 — 4×4 Homogeneous Matrix
3D transforms (translate, rotate, scale, project) are represented as 4×4 matrices using homogeneous coordinates. A point `P` becomes `(Px, Py, Pz, 1)` and a direction `D` becomes `(Dx, Dy, Dz, 0)` — the w=0 makes translations cancel out for directions, which is exactly right for normals and rays.
*Files: all raster files*

#### H3 Model / View / Projection (MVP)
Three matrices are composed once per frame: **Model** rotates the object in world space; **View** positions the camera (transforms world to camera space); **Projection** applies perspective. They are combined as `MVP = Proj × View × Model` and applied in one matrix-vector multiply per vertex. Precomputing MVP saves three separate transforms per vertex.
*Files: all raster files*

#### H4 Perspective Projection Matrix
Maps camera-space coordinates to clip space using `f/aspect` and `f` on the diagonal (where `f = 1/tan(fovy/2)`), and encodes depth into the Z and W components. After the perspective divide (`x/w, y/w, z/w`), coordinates in `[-1,1]` map to the screen. The matrix encodes the entire camera frustum in one 4×4 multiply.
*Files: all raster files*

#### H5 Look-at Matrix
Builds a view matrix from `eye`, `target`, and `up` vectors by constructing an orthonormal right-handed camera frame: `forward = normalize(target - eye)`, `right = normalize(forward × up)`, `up' = right × forward`. The resulting matrix transforms world-space points into camera space. Every raster file's camera is defined this way.
*Files: all raster files*

#### H6 Normal Matrix — Cofactor of Model 3×3
When a model matrix contains non-uniform scale, transforming normals with the model matrix distorts them (normals are no longer perpendicular to the surface). The correct transform is `transpose(inverse(upper-left 3×3))`. For pure rotation matrices the inverse equals the transpose so the normal matrix equals the model matrix — but computing the cofactor handles all cases.
*Files: all raster files*

#### H7 Rotation Matrices (X, Y axes)
`m4_rotate_y(a)` and `m4_rotate_x(a)` build standard Euler rotation matrices. Composing `Ry × Rx` gives a tumbling rotation that shows all faces of the mesh over time without ever getting stuck on one axis — the slightly different X and Y rates prevent periodic synchronisation (gimbal repetition).
*Files: all raster files*

#### H8 Perspective Divide — Clip to NDC
After multiplying by the MVP matrix, coordinates are in clip space with a non-unit W. Dividing by W (`x/w, y/w, z/w`) maps to Normalised Device Coordinates `[-1,1]³`. The screen-space conversion is then `screen_x = (ndcX + 1) / 2 × cols`, `screen_y = (-ndcY + 1) / 2 × rows` (Y is flipped because ncurses row 0 is at the top).
*Files: all raster files*

#### H9 Cross Product & Dot Product
`dot(A,B) = |A||B|cos θ` — used in lighting (`N·L` gives the cosine of the light angle, which equals the Lambertian diffuse term). `cross(A,B)` produces a vector perpendicular to both A and B — used to build the camera right/up vectors in look-at, to compute face normals, and to reconstruct displaced normals.
*Files: all raster and raymarcher files*

---

### I — Raymarching & SDF

#### I1 Signed Distance Functions (SDF)
An SDF returns the signed minimum distance from a point P to a surface: negative inside, positive outside, zero on the surface. The sphere SDF is simply `length(P) - radius`. SDFs can be combined with `min` (union), `max` (intersection), and `-` (subtraction) to build complex shapes analytically, with no mesh required.
*Files: `raymarcher.c`, `raymarcher_cube.c`*

#### I2 Sphere Marching Loop
Cast a ray from the camera. At each step, evaluate the SDF. The SDF tells you the safe distance you can step without crossing any surface — so step by exactly that amount. Near a surface the SDF approaches zero and steps shrink; a hit is declared when the SDF falls below an epsilon (0.002). This is guaranteed safe and converges much faster than fixed-step raytracing.
*Files: `raymarcher.c`, `raymarcher_cube.c`*

#### I3 SDF Normal via Finite Difference
The gradient of an SDF equals the surface normal: `N = normalize(∇SDF(P))`. Approximating the gradient numerically as `(SDF(P+εx̂) - SDF(P-εx̂)) / 2ε` along each axis gives the normal at any point on any SDF without needing an analytic formula. This generalizes to any arbitrary SDF shape.
*Files: `raymarcher_cube.c`*

#### I4 SDF Primitives — Sphere, Box, Torus
- **Sphere:** `length(P) - R`
- **Box:** `length(max(abs(P) - half_extents, 0))` — the `max(..., 0)` clamps inside the box to zero
- **Torus:** `length(vec2(length(P.xz) - R, P.y)) - r` — measures distance to the ring centreline

Each primitive has a closed-form formula and composites with others via simple `min`/`max`.
*Files: `raymarcher.c`, `raymarcher_cube.c`*

---

### J — Software Rasterization

#### J1 Mesh — Vertex & Triangle Arrays
A mesh is two flat arrays: `Vertex[]` (position, normal, UV) and `Triangle[]` (three integer indices into the vertex array). The pipeline iterates triangles, looks up the three vertices by index, and processes them. This separation means vertices can be shared between triangles, saving memory and enabling smooth normal averaging.
*Files: all raster files*

#### J2 UV Sphere Tessellation
Parameterise the sphere surface with longitude `θ ∈ [0, 2π)` and latitude `φ ∈ [0, π]`. Position is `(R·sinφ·cosθ, R·cosφ, R·sinφ·sinθ)`. Normal equals the normalised position for a unit sphere. Poles (`sinφ ≈ 0`) are handled explicitly to avoid degenerate normals. Quads are split into two CCW-wound triangles.
*Files: `sphere_raster.c`, `displace_raster.c`*

#### J3 Torus Tessellation
The torus is parameterised by two angles: `θ` (around the ring) and `φ` (around the tube). Position is `((R + r·cosφ)·cosθ, r·sinφ, (R + r·cosφ)·sinθ)`. The outward tube normal is `normalize(position - ring_centre)` where `ring_centre = (R·cosθ, 0, R·sinθ)`.
*Files: `torus_raster.c`*

#### J4 Cube Tessellation — Flat Normals
Each cube face has four dedicated vertices sharing the same outward face normal — no vertex is shared between faces. Sharing vertices would require averaged normals, which rounds the corners. Flat per-face normals make every fragment on a face receive identical diffuse lighting, producing the hard-edged look that defines a cube.
*Files: `cube_raster.c`*

#### J5 Vertex Shader — VSIn / VSOut
A vertex shader is a C function `void vert(const VSIn *in, VSOut *out, const void *uni)` called once per triangle vertex. It receives a model-space position and normal and must output a clip-space `Vec4 clip_pos` plus any per-vertex data (`world_pos`, `world_nrm`, `custom[4]`) that will be interpolated across the triangle for the fragment shader.
*Files: all raster files*

#### J6 Fragment Shader — FSIn / FSOut
A fragment shader is called once per rasterized pixel. It receives the barycentrically-interpolated vertex outputs (`world_pos`, `world_nrm`, UV, `custom[4]`) plus the screen cell coordinates. It outputs a `Vec3 color` and a `bool discard`. Setting `discard = true` makes the pipeline skip writing this pixel — used by the wireframe shader to remove interior fragments.
*Files: all raster files*

#### J7 ShaderProgram — Split vert_uni / frag_uni
The vertex and fragment shaders can need different uniform struct types (e.g., `vert_displace` needs `DisplaceUniforms` for the displacement function pointer, while `frag_toon` needs `ToonUniforms` for the band count). A single `void *uniforms` pointer forces one shader to cast to the wrong type, causing a segfault when it dereferences a field that doesn't exist. Two separate pointers — `vert_uni` and `frag_uni` — give each shader exactly what it needs.
*Files: all raster files*

#### J8 Barycentric Coordinates
Given a triangle with screen-space vertices `V0, V1, V2`, any point `P` inside the triangle can be written as `P = b0·V0 + b1·V1 + b2·V2` where `b0 + b1 + b2 = 1` and all `bᵢ ≥ 0`. The coefficients `(b0, b1, b2)` are barycentric coordinates. They are computed from 2D cross products of the triangle's edges and serve as the weights for interpolating any per-vertex attribute.
*Files: all raster files*

#### J9 Barycentric Interpolation of Vertex Attributes
Any attribute stored at the three triangle vertices (color, normal, UV, custom payload) can be smoothly interpolated across the triangle's interior by computing the weighted sum `attr = b0·attr0 + b1·attr1 + b2·attr2`. The barycentric weights automatically ensure the interpolated value matches each vertex exactly at the corners and blends linearly in between.
*Files: all raster files*

#### J10 Z-buffer (Depth Buffer)
A `float zbuf[cols × rows]` stores the depth of the closest fragment seen so far at each pixel, initialised to `FLT_MAX`. Before writing a fragment, compare its interpolated depth `z` against `zbuf[idx]`; if `z >= zbuf[idx]` the fragment is behind something already drawn and is discarded. This correctly handles overlapping geometry without sorting triangles.
*Files: all raster files*

#### J11 Back-face Culling — Screen-space Signed Area
After projecting triangle vertices to screen space, compute the 2D signed area: `area = (sx1-sx0)×(sy2-sy0) - (sx2-sx0)×(sy1-sy0)`. CCW-wound front faces have positive area; CW-wound back faces have negative area. Discarding `area ≤ 0` triangles halves the rasterization work for closed meshes and is free after the perspective divide.
*Files: all raster files*

#### J12 Near-plane Clip Reject
If all three vertices of a triangle have `clip_pos.w < 0.001`, the entire triangle is behind the camera's near plane and should be skipped. Without this check, vertices behind the camera undergo a perspective divide with a near-zero or negative W, projecting to garbage screen coordinates and producing huge corrupt triangles.
*Files: all raster files*

#### J13 Framebuffer — zbuf + cbuf
The raster pipeline writes to two CPU arrays rather than directly to ncurses. `zbuf` is the depth buffer; `cbuf` is an array of `Cell{ch, color_pair, bold}`. Only after the full frame is rasterized does `fb_blit()` iterate `cbuf` and call `mvaddch` for non-empty cells. This separates the rendering math from the ncurses I/O and makes the pipeline easier to reason about.
*Files: all raster files*

#### J14 Barycentric Wireframe
Assign each vertex of every triangle a unique barycentric identity vector: `(1,0,0)`, `(0,1,0)`, `(0,0,1)`. After interpolation, every interior fragment has all three components strictly positive; fragments near an edge have one component close to zero. Testing `min(b0,b1,b2) < threshold` in the fragment shader identifies edge fragments with no geometry queries — it works for any triangle shape.
*Files: all raster files (wireframe shader)*

#### J15 Vertex Displacement
Before transforming to clip space, the vertex position is moved along its surface normal by a scalar `d` from a displacement function: `displaced_pos = pos + normal × d`. The displacement function can be any mathematical expression in position and time. This deforms the mesh every frame in the vertex shader, creating animated surface waves with no mesh rebuild.
*Files: `displace_raster.c`*

#### J16 Central Difference Normal Recomputation
After displacing a vertex, the original normal is wrong — it pointed perpendicular to the undisplaced sphere, not the deformed surface. Recompute it numerically: sample the displacement function at `pos ± ε` along two tangent directions, compute how much the surface rises/falls (`d_t`, `d_b`), reconstruct the displaced tangent vectors, and take their cross product. This works for any displacement function without needing an analytic derivative.
*Files: `displace_raster.c`*

#### J17 Tangent Basis Construction
To step along the surface for central differences, you need two vectors tangent to the surface. Given normal `N`, pick an "up" reference that is not parallel to `N` (swap between `(0,1,0)` and `(1,0,0)` when `N` is nearly vertical). Then `T = normalize(up × N)` and `B = N × T`. This gives an orthonormal frame `(T, B, N)` that lies in the surface plane.
*Files: `displace_raster.c`*

---

### K — Shading Models

#### K1 Blinn-Phong Shading
Computes lighting as `ambient + diffuse + specular`. Diffuse = `max(0, N·L)` (Lambertian) where `L` is the normalised direction to the light. Specular = `max(0, N·H)^shininess` where `H = normalize(L + V)` is the half-vector between light and view directions (Blinn's approximation — cheaper than reflecting L). The shininess exponent controls highlight size.
*Files: all raster files, `raymarcher.c`*

#### K2 Toon / Cel Shading — Banded Diffuse
Quantise the diffuse term into N discrete bands: `banded = floor(diff × N) / N`. This replaces the smooth gradient with hard steps, giving the flat-coloured look of cel animation. A binary specular (`N·H > 0.94 ? 0.7 : 0`) adds a hard highlight. On the cube, each flat face falls entirely into one band, making the effect especially striking.
*Files: all raster files*

#### K3 Normal Visualisation Shader
Map world-space normals from `[-1,1]` to `[0,1]`: `color = N × 0.5 + 0.5`. This encodes surface orientation as RGB: +X right = red, +Y up = green, +Z toward camera = blue. It is invaluable for debugging: correct normals produce smooth colour gradients; wrong normals show as sudden hue jumps or flat solid colours.
*Files: all raster files*

#### K4 Parametric Torus Lighting (Donut)
The `donut.c` algorithm computes 3D positions by rotating a point on a circle (the tube cross-section) around the Y axis, applies two rotation matrices (A and B for the tumble), then perspective-projects. The luminance is computed from the surface normal dotted with a fixed light direction — no matrix pipeline, just trigonometry unrolled by hand.
*Files: `donut.c`*

---

### L — Algorithms & Data Structures

#### L1 Bresenham Line Algorithm
Draws a line between two integer grid points by incrementally tracking the sub-pixel error. At each step it moves in the major axis direction and conditionally steps in the minor axis when the accumulated error exceeds half a pixel. It uses only integer addition and comparison — no floating-point per step — making it the fastest possible discrete line rasterizer.
*Files: `wireframe.c`, `spring_pendulum.c`*

#### L2 Ring Buffer
A fixed-size array with `head` and `tail` indices that wrap modulo the array size. `matrix_rain.c` uses a ring buffer for each column's trail: the `head` index advances each tick, overwriting the oldest entry. Reading backwards from head gives the trail in order from newest to oldest without any shifting or allocation.
*Files: `matrix_rain.c`, `flowfield.c`, `complex_flowfield.c`*

#### L3 Z-buffer / Depth Sort
Both the z-buffer (raster) and z-sorting (donut) solve the visibility problem: when multiple objects project onto the same pixel, which one is in front? The z-buffer stores per-pixel depth and discards farther fragments. The donut sorts all lit points by depth and iterates front-to-back, painting over earlier results with closer ones.
*Files: all raster files, `donut.c`*

#### L4 Bounding Box Rasterization
Rather than testing every pixel on screen against every triangle, compute the axis-aligned bounding box of the projected triangle and iterate only those pixels. The box is clamped to `[0, cols-1] × [0, rows-1]`. This reduces the inner loop from `cols × rows` iterations to roughly the triangle's screen area — critical for performance.
*Files: all raster files*

#### L5 Fisher-Yates Shuffle
To randomise the column scan order in `sand.c` each tick: fill an array `[0..cols-1]`, then for `i` from `cols-1` down to 1, swap `arr[i]` with `arr[rand() % (i+1)]`. Each permutation is equally likely, removing the left/right scan bias that would otherwise cause sand to pile up asymmetrically. This is the standard O(n) unbiased shuffle.
*Files: `sand.c`*

#### L6 Callback / Function Pointer Patterns
`brust.c` passes a `scorch` function pointer into `burst_tick`; the raster pipeline invokes `sh->vert` and `sh->frag` through `ShaderProgram`; `flowfield.c` maps colours through a theme function. Function pointers turn hardcoded behaviour into pluggable strategies — new displacement modes, new shaders, new themes — without touching the pipeline code.
*Files: `brust.c`, all raster files*

#### L7 Lookup Table (LUT)
Precomputing an array of results and indexing into it at runtime turns repeated expensive computations into a single memory access. `fire.c` precomputes the decay table; `raymarcher.c` precomputes the ASCII character ramp; `aafire_port.c` precomputes the per-row heat decay value. LUTs trade memory for speed and are especially valuable inside inner loops.
*Files: `fire.c`, `aafire_port.c`, `raymarcher.c`*

#### L8 Bilinear Interpolation
Sampling a 2D grid at a non-integer position by weighting the four surrounding grid cells: `lerp(lerp(top-left, top-right, fx), lerp(bottom-left, bottom-right, fx), fy)`. `flowfield.c` and `complex_flowfield.c` both use bilinear interpolation to sample the angle grid at continuous particle positions, producing smooth velocity fields rather than blocky step functions.
*Files: `flowfield.c`, `complex_flowfield.c`*

---

### M — Systems Programming

#### M1 Signal Handling — sig_atomic_t
Signal handlers can interrupt any instruction in the main loop. Writing a multi-byte type from a handler can produce a torn read in the main loop. `volatile sig_atomic_t` is the only type the C standard guarantees can be read and written atomically from a signal handler. `volatile` prevents the compiler from caching the value in a register and missing the handler's write.
*Files: all files*

#### M2 atexit — Guaranteed Cleanup
`atexit(cleanup)` registers a function to run when `exit()` is called for any reason — including `abort()`, falling off `main()`, or an uncaught signal that calls `exit()`. `cleanup()` calls `endwin()`, which restores the terminal. Without this, a crash leaves the terminal in raw mode with echo disabled and the cursor hidden.
*Files: all files*

#### M3 nanosleep — High-resolution Sleep
`nanosleep(&req, NULL)` sleeps for the specified `timespec` duration, providing sub-millisecond sleep resolution (typically ~100 µs granularity on Linux). The `if(ns <= 0) return` guard before the call prevents passing a negative duration — which has undefined behaviour — when the frame was already over budget.
*Files: all files*

#### M4 Static Global App — Signal Handler Reach
Signal handlers have no arguments beyond the signal number. To reach application state (`g_app.running = 0`), the `App` struct is declared as a static global. The handler accesses it by name. This is the standard C pattern — the global must be `static` to limit its visibility to the translation unit and prevent name collisions.
*Files: all files*

#### M5 void* Uniform Casting — Type-safe Shader Interface
Shader functions accept `const void *uni` and cast it to the specific struct they need. The cast is safe because the caller (`ShaderProgram`) stores the pointer at construction time and the type is guaranteed by the `scene_build_shader` logic. The `void*` parameter makes the function pointer signature uniform for all shaders regardless of which uniform struct they use.
*Files: all raster files*

#### M6 memset for Zero-init
`memset(s, 0, sizeof *s)` zeroes every byte of a struct at once, including padding bytes. This is faster than a designated initializer for large structs and ensures no field is left uninitialised. In C, zeroing a float gives `0.0f`, zeroing a pointer gives `NULL`, and zeroing a bool gives `false` — reliably, because IEEE 754 zero is all-zero bits.
*Files: all raster files, `bounce.c`*

#### M7 size_t Casts in malloc
`malloc((size_t)(n) * sizeof(T))` — the `size_t` cast is critical when `n` is a signed `int`. If `n` is large, `n * sizeof(T)` overflows `int` (signed integer overflow is undefined behaviour in C) before the implicit conversion to `size_t` happens. Casting first makes the multiplication happen in unsigned 64-bit, preventing both UB and silent underallocation.
*Files: all raster files, `flowfield.c`, `complex_flowfield.c`, `sand.c`*

---

---

### A9 use_default_colors — Transparent Terminal Background

`use_default_colors()` (called immediately after `start_color()`) allows `-1` to be passed as the background color in `init_pair`. A pair with background `-1` uses the terminal emulator's own background — transparent, image, blur, whatever it is. Without calling `use_default_colors()` first, a `-1` background silently defaults to `COLOR_BLACK`.
*Files: `bonsai.c`, `matrix_rain.c`*

#### A10 ACS Line-drawing Characters

ncurses provides portable box-drawing glyphs through `ACS_*` macros: `ACS_ULCORNER` (┌), `ACS_URCORNER` (┐), `ACS_LLCORNER` (└), `ACS_LRCORNER` (┘), `ACS_HLINE` (─), `ACS_VLINE` (│). On VT100 terminals they activate the alternate character set; on modern UTF-8 terminals they emit Unicode box-drawing code points. Always prefer `ACS_*` over hardcoded characters for maximum terminal compatibility.
*Files: `bonsai.c` (message panel)*

#### A11 (chtype)(unsigned char) — Sign-extension Guard

`char` is signed on most platforms. High-value ASCII characters (128–255) have bit 7 set. When cast directly to `chtype` (unsigned long), they sign-extend to large values that ncurses interprets as attribute flags, producing garbage rendering. The fix is a double cast: `(chtype)(unsigned char)ch` — `unsigned char` zero-extends to 8 bits, then `chtype` preserves that non-negative value. Every `mvwaddch` call in this project uses this cast.
*Files: all files*

#### A12 attr_t — Composing Color + Attribute in One Variable

```c
attr_t attr = COLOR_PAIR(p->hue);
if (p->life > 0.65f) attr |= A_BOLD;
```

Building the attribute bitmask into an `attr_t` variable before calling `attron()` allows conditional logic without nested calls. The result is passed as a single OR to `wattron(w, attr)`. All attribute flags (`A_BOLD`, `A_DIM`, `A_REVERSE`, `A_UNDERLINE`, `A_BLINK`, `A_ITALIC`) live in the upper bits of the `chtype`/`attr_t` word; `COLOR_PAIR(n)` occupies a different bit region, so the OR is always safe.
*Files: `brust.c`, `aafire_port.c`, `constellation.c`*

#### A13 Dynamic Color — Re-registering Pairs Mid-animation

`init_pair()` can be called at any time during the animation loop, not just during initialization. The new colors take effect on the next `doupdate()`. `flocking.c` uses this to implement a cosine palette that smoothly cycles all three flock colors by computing new RGB values each tick and re-registering the corresponding pair:

```c
/* xterm-256 color cube: index = 16 + 36*r + 6*g + b, r/g/b in [0,5] */
int cube_idx = 16 + 36*ri + 6*gi + bi;
init_pair(pair_num, cube_idx, COLOR_BLACK);
```

This is the only way to animate color in ncurses without changing the scene content — the pair mapping changes while the character data stays the same.
*Files: `flocking.c`*

---

### G7 Stippled Lines — Distance-fade Bresenham

A stippled line draws only every Nth pixel of a Bresenham line, controlled by a `stipple` parameter:

```c
if (step_count % stipple == 0) { /* draw this cell */ }
step_count++;
```

`stipple = 1` draws every cell (solid), `stipple = 2` draws every other cell (dotted). `constellation.c` uses this to encode connection distance: close star pairs get bold solid lines, far pairs get dimmer dotted lines. The effect reads as continuous fading even though ncurses has no alpha blending — it exploits the fact that the eye integrates the density of a dotted line into a perceived brightness.
*Files: `constellation.c`*

#### G8 cell_used Grid — Line Deduplication

When multiple Bresenham lines cross the same terminal cell, the last writer wins — producing a jumbled mix of line characters in dense regions. `constellation.c` prevents this with a per-frame boolean VLA:

```c
bool cell_used[rows][cols];   /* stack-allocated, zeroed each frame */
memset(cell_used, 0, sizeof cell_used);
/* In Bresenham loop: */
if (!cell_used[y][x]) { draw(y, x, ch); cell_used[y][x] = true; }
```

The first line to visit a cell claims it; all others silently skip. Dense connection clusters render as smooth individual lines rather than a chaotic character pile.
*Files: `constellation.c`*

#### G9 Scorch Mark Persistence

`brust.c` maintains a `scorch[]` array that persists between explosion cycles. When a particle lands, its character and position are saved into the scorch array. Each frame, scorch marks are drawn with `A_DIM` (faded appearance) before drawing active particles. This creates visual history without any image compositing — the screen accumulates past explosions as progressively dimmer marks.
*Files: `brust.c`*

#### G10 Bright Hue-varying Theme Palette

A theme palette that ramps from **dark to bright** within one hue has a fatal flaw on dark terminals: the low-gradient steps are near-black and invisible against the background. The scene appears partially rendered even when correct.

The fix: make every palette entry a vivid, saturated color by ensuring at least one RGB channel sits at 4 or 5 in the xterm-256 cube (`color = 16 + 36r + 6g + b`, r/g/b ∈ 0–5). Vary **hue** across the gradient instead of brightness. Every step is readable; the theme's identity comes from its hue family (fire=red-orange, arctic=cyan-blue, etc.).

```c
/* Bad: dark → light of one hue — low steps invisible */
{17, 18, 19, 20, 27, 33, 45, 159}  /* near-black blues at indices 0-3 */

/* Good: hue varies, all steps bright */
{51, 87, 123, 159, 153, 189, 225, 231}  /* cyan→sky→lavender→white, all visible */
```

Verification: decode any xterm-256 index to RGB: `r=(c-16)/36`, `g=((c-16)%36)/6`, `b=(c-16)%6`. An entry is "bright enough" when `max(r,g,b) >= 4`.
*Files: `raymarcher/sdf_gallery.c`, `raymarcher/mandelbulb_explorer.c`*

---

### N — Flocking & Collective Behaviour

#### N1 Classic Boids — Separation / Alignment / Cohesion

Reynolds' three rules, each computed over neighbors within `PERCEPTION_RADIUS`:

- **Separation** — steer away from neighbors that are too close (`dist < SEPARATION_RADIUS`). Force magnitude grows as `1/dist` so it spikes near contact.
- **Alignment** — accumulate the average velocity of all neighbors; steer toward that average direction.
- **Cohesion** — compute the centroid of all neighbors; steer toward it.

The three force vectors are combined with independent weights (`SEP_WEIGHT`, `ALG_WEIGHT`, `COH_WEIGHT`). The boid's steering force is added to its velocity, then clamped to `MAX_SPEED`.
*Files: `flocking.c`*

#### N2 Vicsek Model — Noise-driven Phase Transition

The Vicsek model is a minimal self-propulsion model that exhibits a phase transition between ordered (flock) and disordered (swarm) states as noise increases:

```
θ_i(t+1) = <θ_neighbors>(t) + η × ξ
```

where `<θ_neighbors>` is the average heading of all boids within radius, `η` is the noise amplitude, and `ξ` is uniform `[−π, π]` noise. As `η` increases beyond a critical value, the global order parameter (average heading alignment) collapses from 1 to 0 — the flock fragments. Adjustable with `n/m` keys.
*Files: `flocking.c`*

#### N3 Leader Chase

Each follower computes the toroidal vector to its flock leader and steers toward it with a proportional force. The proportionality constant controls responsiveness — too high produces jitter, too low produces a slow drift. The leader itself wanders using a random-walk heading perturbation each tick.
*Files: `flocking.c`*

#### N4 Orbit Formation

Followers maintain a target angular position on a ring of radius `ORBIT_RADIUS` around the leader. Each tick, the target angle advances by a fixed angular velocity, so followers circle the leader in formation. The steering force is proportional to the distance from the follower's current position to its target position on the ring.
*Files: `flocking.c`*

#### N5 Predator-Prey

Flock 0 is the predator: its steering force targets the nearest boid from flocks 1–2. Flocks 1–2 are prey: each of their boids computes the toroidal vector to flock 0's leader and steers away from it. The same perception radius governs both chase and flee decisions. When the predator is far, prey runs Classic Boids internally.
*Files: `flocking.c`*

#### N6 Toroidal Topology — Wrap-around Physics

All flocking modes use toroidal (wrap-around) boundaries instead of elastic wall bounces. Position wraps:

```c
if (b->px < 0)       b->px += max_px;
if (b->px > max_px)  b->px -= max_px;
```

Distance and steering use the shortest toroidal path:

```c
float d = a - b;
if (d >  max * 0.5f) d -= max;
if (d < -max * 0.5f) d += max;
```

This produces `|d| ≤ max/2`, the correct shortest-path distance across the wrap boundary. All neighbor detection, cohesion, and proximity brightness depend on this function — using raw `a - b` would produce wrong steering forces for boids near opposite edges.
*Files: `flocking.c`*

---

### O — Procedural Growth

#### O1 Recursive Branch Growth — Bonsai

`bonsai.c` grows a tree one step per tick from a pool of active branches. Each branch struct carries `(x, y, dx, dy, life, type, color)`. Each tick, `branch_step()` draws the character at `(y, x)` (chosen by slope), optionally spawns child branches when life crosses a threshold, and decrements life. Branches with `life ≤ 0` are retired.

The character drawn at each step is chosen by the slope of `(dx, dy)`:
```c
if      (fabs(dx) < fabs(dy) / 2)  ch = '|';
else if (fabs(dy) < fabs(dx) / 2)  ch = '-';
else if (dx * dy > 0)               ch = '\\';
else                                ch = '/';
```

Growth simulation runs at `SIM_FPS`; each sim tick produces one step per active branch — visually one character per branch per tick.
*Files: `bonsai.c`*

#### O2 Branch Type State Machine

Four branch types drive different wander and branching behaviour:

| Type | Tendency | Branch Spawning |
|---|---|---|
| `TRUNK` | Upward, moderate wander | Spawns `SHOOT` branches frequently |
| `SHOOT` | Outward / upward diagonal | Spawns `DYING` at low life |
| `DYING` | Gravitational sag | No new branches |
| `DEAD` | No wander | No branches |

Type is set at spawn and never changes. The type-specific code path is a simple `switch(b->type)` in `branch_step()`. Five overall tree styles (`RANDOM`, `DWARF`, `WEEPING`, `SPARSE`, `BAMBOO`) adjust the initial parameters and spawning thresholds.
*Files: `bonsai.c`*

#### O3 Leaf Scatter

When a branch exhausts its life, `leaf_scatter()` places leaf characters in a Gaussian blob around the tip:

```c
for (int i = 0; i < n_leaves; i++) {
    int lx = tip_x + (int)(gaussian() * LEAF_SPREAD);
    int ly = tip_y + (int)(gaussian() * LEAF_SPREAD / 2.0f);
    /* half spread on Y — terminal cells are taller than wide */
    draw_char(ly, lx, leaf_char(), COLOR_PAIR(leaf_color) | A_BOLD);
}
```

The Y spread is halved because terminal cells are twice as tall as wide — equal pixel spread requires half as many cell rows as columns. Leaf chars are drawn with `A_BOLD` to appear lighter and more delicate than branch chars.
*Files: `bonsai.c`*

---

---

### P — Fractal Systems

#### P1 Diffusion-Limited Aggregation (DLA)

DLA simulates the formation of fractal crystal structures. A seed cell is frozen at the centre. Random walkers are released one at a time and each takes a random walk until it lands adjacent to a frozen cell, at which point it freezes and extends the aggregate. Repeating this millions of times produces a branching, dendritic structure whose fractal dimension is approximately 1.71.

The terminal grid stores a `uint8_t` color index per cell (0 = empty). Walker movement is one cell per tick in a random cardinal (or diagonal) direction. The adjacency check scans the 4 or 8 neighbours of the walker's current cell.

Key design decisions:
- **Respawn radius** — if a walker escapes beyond `max_dist + SPAWN_MARGIN` it is teleported to a random point on the spawn circle. This prevents walkers from wandering away forever.
- **N_WALKERS concurrent** — running multiple walkers simultaneously speeds up growth without changing the statistical fractal dimension.
- **Sticking probability < 1.0** — probabilistic sticking (coral.c) smooths the branches and allows side growth.

*Files: `snowflake.c`, `coral.c`*

#### P2 D6 Hexagonal Symmetry with Aspect Correction

Snowflakes have D6 symmetry: 6-fold rotational symmetry plus a reflection axis. `snowflake.c` enforces this by freezing all 12 D6 images of every new point simultaneously. When a walker freezes at `(col, row)`:

```
for k in 0..5:           /* 6 rotations */
    for mirror in [+1,-1]: /* 2 reflections */
        plot D6-image of (col, row)
```

Terminal cells are non-square. A naïve integer rotation mixes cell-space distances with Euclidean angles, distorting the symmetry. The fix:

1. Convert displacement to Euclidean: `dy_e = dy / ASPECT_R`
2. Apply 2-D rotation in Euclidean space
3. Convert result back to cell space: `new_row = cy + ry_e * ASPECT_R`

Only after this correction do the 6 rotated images appear truly 60° apart on screen.

*Files: `snowflake.c`*

#### P3 Anisotropic DLA — Directional Sticking

Standard DLA is isotropic — branches grow equally in all directions. `coral.c` breaks this by making sticking probability depend on the relative direction of the approaching walker:

```c
float stick_prob;
if      (dy == -1) stick_prob = 0.90f;  /* walker came from above  */
else if (dy ==  0) stick_prob = 0.40f;  /* walker came from side   */
else               stick_prob = 0.10f;  /* walker came from below  */
if ((float)rand() / RAND_MAX > stick_prob) continue;  /* skip */
```

Walkers approaching from above stick with 90% probability (downward-growing branches are easy to cap). Walkers approaching from the sides stick at 40% (side branches form but are selective). Walkers from below rarely stick at 10% (almost no downward growth). Combined with a downward walker drift and 8 bottom seeds, the result is upward-growing coral branches.

*Files: `coral.c`*

#### P4 Chaos Game / Iterated Function System (IFS)

An IFS is a finite set of affine transforms. The chaos game applies a randomly chosen transform to the current point at each step. After a warm-up period (to move the point onto the attractor), plotting each subsequent point traces the fractal attractor of the IFS.

For the Sierpinski triangle the three transforms are `T_i(x,y) = ((x + V_i.x)/2, (y + V_i.y)/2)` — midpoint between the current point and vertex `i`. After 20 warm-up steps (discarded), the orbit never leaves the attractor. Color tracks the index of the last chosen transform, giving each sub-triangle its own hue.

The attractor is reached because each transform is contractive (Lipschitz constant 0.5). Banach's fixed-point theorem guarantees that repeated application converges to the unique fixed-point set — the Sierpinski triangle.

*Files: `sierpinski.c`, `fern.c`*

#### P5 Barnsley Fern — 4-Transform IFS

The Barnsley Fern uses four affine transforms with non-uniform probabilities:

| Transform | Probability | Effect |
|-----------|-------------|--------|
| `f1` stem  | 1%          | Maps everything to a thin vertical stem |
| `f2` main  | 85%         | Self-similarity — maps fern to slightly smaller fern |
| `f3` left  | 7%          | Maps to lower-left leaflet |
| `f4` right | 7%          | Maps to lower-right leaflet |

The 85% main transform creates the recursive self-similarity: each frond looks like the whole fern at a smaller scale. The two 7% transforms add the alternating side leaflets. The 1% stem maps to the base midrib.

The fern's natural coordinate space spans `x ∈ [−2.5, 2.5]`, `y ∈ [0, 10]` — a 4:1 aspect in math units. On a terminal with ASPECT_R=2, naive uniform scaling would produce a fern only ~9 columns wide. Independent x/y scales map fern math space to a comfortable fraction of the terminal:

```c
scale_y = (rows - 3) / (y_max - y_min)
scale_x = cols * 0.45 / (x_max - x_min)
```

*Files: `fern.c`*

#### P6 Escape-time Iteration — Julia and Mandelbrot Sets

Both sets use the iteration `z_n+1 = z_n² + c` in the complex plane. The two parameters differ:

| Set       | `z₀`         | `c`            |
|-----------|--------------|----------------|
| Julia     | pixel coord  | fixed constant |
| Mandelbrot | 0           | pixel coord    |

A point is in the set if `|z_n|` never exceeds 2 for all n. In practice, the iteration stops at MAX_ITER and the escape ratio `frac = iter / MAX_ITER` is used for coloring.

The escape-time coloring maps `frac` to color bands:
- Points that escape quickly (high `frac`) are near the boundary — these receive the brightest colors.
- Points deep inside the set (low `frac`) remain black.
- The transition bands create the characteristic halo of color around the set boundary.

Julia sets visualise the filled Julia set for a particular `c`. Different `c` values produce dramatically different shapes: `c = −0.7 + 0.27i` gives the Douady rabbit, `c = −0.8 + 0.156i` gives a dendrite, `c = −0.7269 + 0.1889i` gives a seahorse.

*Files: `julia.c`, `mandelbrot.c`*

#### P7 Fisher-Yates Random Pixel Reveal

Computing all pixels in row-major order produces a visible scan line that sweeps downward — aesthetically poor. The Fisher-Yates (Knuth) shuffle produces a uniformly random permutation of all pixel indices:

```c
for (int i = n - 1; i > 0; i--) {
    int j = rand() % (i + 1);
    int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
}
```

Each tick, `PIXELS_PER_TICK` pixels from the shuffled order are computed and plotted. The fractal appears to materialise from scattered noise — far more visually engaging than a scan line. The shuffle is O(n) at init and O(1) per pixel during draw.

The shuffled index array is stored in the Grid struct: `order[GRID_ROWS_MAX * GRID_COLS_MAX]`. At ~80×300 = 24,000 entries × 4 bytes = 96 KB — large enough to declare as a static global, avoiding stack overflow.

*Files: `julia.c`, `mandelbrot.c`*

#### P8 Koch Snowflake — Midpoint Subdivision

The Koch snowflake is constructed by iteratively replacing each segment with four sub-segments. Given segment P→Q:

1. A = P + (Q−P)/3
2. B = P + 2(Q−P)/3
3. M = A + R(+60°)(B−A)   ← the outward bump

`R(+60°)` is the 2-D rotation matrix for 60°:
```
[ cos60  -sin60 ]   [ 0.5    -0.866 ]
[ sin60   cos60 ] = [ 0.866   0.5   ]
```

The choice of R(+60°) vs R(−60°) determines whether M is inside or outside the original triangle. With a clockwise-wound starting triangle and R(+60°), M lies on the outside on all three edges, producing the canonical outward bumps.

Level n has 3 × 4ⁿ segments. Level 5 has 3072 segments, each 1/243 of the circumradius. Bresenham rasterizes each segment to the terminal grid as a single-cell-wide path. At level 5 many segments are sub-pixel, producing single-cell marks — which is fine; the shape is still recognizable.

*Files: `koch.c`*

#### P9 Recursive Tip Branching — Lightning

Lightning grows as a fractal binary tree rather than a random walk. Each active tip carries a `lean` bias (integer ∈ {−2, −1, 0, 1, 2}) that persists across steps. Each tick:

1. The tip advances one row downward
2. Column offset = lean / 2 (rounded), so lean ±2 gives ±1 cell/step diagonal
3. Character selected by movement direction: `|` straight, `/` lean left, `\` lean right
4. After `MIN_FORK_STEPS` steps, with probability `FORK_PROB`, the tip forks into two children with lean ± 1

This produces a fractal binary tree that spreads as it descends — each child's lean diverges from the parent by ±1, so branches spread apart at a controlled rate. The structure is deterministic given a seed but visually complex enough to look organic.

The glow halo is generated at draw time (not stored in the grid): for each frozen cell, Manhattan neighbors at distance 1 and 2 are drawn with dim attributes if they are empty.

*Files: `lightning.c`*

#### P10 Buddhabrot — Density Accumulator Fractal

The Buddhabrot is a rendering of the Mandelbrot iteration that illuminates orbital trajectories rather than escape-time values. For each randomly sampled complex number c that escapes (|z| exceeds 2 within max_iter steps), every point visited by its orbit is projected onto a 2-D hit-count grid. Cells traversed by many orbits accumulate large counts and glow brightest in the final image — producing a figure that resembles a seated Buddha.

**Two-pass sampling:**

```
Pass 1 (escape test):  iterate z→z²+c from z=0; record whether c escapes
Pass 2 (orbit trace):  if condition met, iterate again and increment counts[row][col]
```

The two-pass design avoids allocating an orbit buffer. Pass 1 is cheap (just a boolean result); pass 2 is only run for orbits that satisfy the mode condition.

**Anti-Buddhabrot** traces orbits that do NOT escape (bounded orbits), illuminating the Mandelbrot interior structure rather than the outer filaments.

**Cardioid/bulb pre-rejection** skips samples inside the main cardioid and period-2 bulb in Buddha mode — these never escape, so they waste iterations without contributing:

```c
float q = (cr - 0.25f) * (cr - 0.25f) + ci * ci;
if (q * (q + cr - 0.25f) < 0.25f * ci * ci) return;  /* cardioid */
if ((cr+1.0f)*(cr+1.0f) + ci*ci < 0.0625f)   return;  /* period-2 bulb */
```

**Dynamic range problem and log normalization:**
Anti-mode attractor cells accumulate up to `max_iter × accepted_samples` hits (millions). With sqrt normalization, every transient cell (count=1) maps to sqrt(1/10⁶)≈0.001 — still in the first visible band — causing dots scattered across a blank screen. Log normalization `t = log(1+count)/log(1+max_count)` compresses this: a transient cell with count=1 maps to log(2)/log(10⁶+1)≈0.035, below the invisible floor.

**Mode-aware invisible floor:**
Buddha mode has far lower max_count, so the floor is kept low (0.05) to preserve fine structure. Anti mode uses a high floor (0.25) to suppress noise dots:

```c
float floor = anti ? 0.25f : 0.05f;
if (t < floor) return 0;  /* invisible */
```

**Five presets:** buddha 500, buddha 2000 (outer filaments), anti 100, anti 500, anti 1000 (interior complexity). Each preset accumulates TOTAL_SAMPLES=150000 random samples then holds for DONE_PAUSE_TICKS before cycling.

*Files: `buddhabrot.c`*

#### P11 Pascal-Triangle Bat Swarms — Artistic Formation Flying

`bat.c` renders three groups of ASCII bats flying outward from the terminal centre in formation. Each group is a filled Pascal triangle with the leader at the apex and subsequent rows filling with increasing numbers of bats.

**Formation indexing (bat_form_offset):**
Given flat bat index k, row r is the largest r where r*(r+1)/2 ≤ k:
- Row 0: 1 bat (leader)
- Row 1: 2 bats
- Row r: r+1 bats
- Total for n_rows rows: (n_rows+1)*(n_rows+2)/2

Position within the formation is computed as:
```c
int r = 0;
while ((r+1)*(r+2)/2 <= k) r++;
int pos = k - r*(r+1)/2;
*along = -(float)r * LAG_PX;           /* behind leader */
*perp  = ((float)pos - (float)r * 0.5f) * SPREAD_PX;  /* left-right spread */
```

The along/perp offsets are rotated into world space using the group's flight angle: `world_x = along*cos(angle) - perp*sin(angle)`.

**Wing animation:** four-frame cycle `/ - \ -` for left wing, mirrored for right wing. Body character is always `o`. The wing phase advances each tick for all bats in a group simultaneously.

**Live resize:** `+`/`-` keys change n_rows (1–6). New bats are placed in formation relative to the current leader position without interrupting flight. Maximum bats per group = (6+1)*(6+2)/2 = 28.

**Three groups** with staggered launch (30 ticks apart) and distinct colors: light purple (xterm 141), electric cyan (xterm 87), pink-magenta (xterm 213). Six preset launch angles (330°, 210°, 90°, 45°, 135°, 270°) distribute groups in different directions each cycle.

*Files: `bat.c`*

---

#### P12 Wa-Tor Predator-Prey Simulation

`wator.c` simulates Alexander Dewdney's Wa-Tor ocean. Fish breed and drift randomly; sharks hunt fish and starve without food. The interaction produces emergent Lotka-Volterra-style population oscillations visible in a dual histogram at the bottom of the screen.

**Grid state:** Four `uint8_t` arrays: `g_type` (EMPTY/FISH/SHARK), `g_breed` (age or breed counter), `g_hunger` (shark starvation timer), `g_moved` (double-process guard). `g_moved[r][c] = 1` is set when an entity moves into or acts on that cell; the cell is skipped when encountered again in the same tick's processing loop.

**Fisher-Yates processing order:** All `rows × cols` indices are shuffled before each tick. Without shuffling, top-left processing order creates directional movement bias — fish drift right/down preferentially. The shuffle makes movement statistically isotropic at O(n) cost per tick.

**Fish step:** increment age; find random empty neighbour; move there (mark `g_moved`); if age ≥ `FISH_BREED`, leave newborn at origin (not destination — newborn won't act this tick since origin was already processed).

**Shark step:** increment breed and hunger; if `hunger > SHARK_STARVE`, die; else prefer fish neighbours (eat, reset hunger) over empty neighbours (just move); if `breed ≥ SHARK_BREED`, spawn child at origin.

**Key insight:** Placing children at the origin (departure cell) rather than the destination (arrival cell) is the rule that prevents newborns from acting in the same tick as they are born — origin was already processed, destination may not have been.

*Files: `artistic/wator.c`*

---

#### P13 Abelian Sandpile — Self-Organised Criticality

`sandpile.c` implements the Bak-Tang-Wiesenfeld model: grains drop one at a time onto the centre; any cell with ≥ 4 grains topples (loses 4, gives 1 to each cardinal neighbour). Avalanches propagate via BFS until the grid is stable.

**BFS queue with `g_inq` deduplication:**
```c
static void enq(int r, int c) {
    if (g_inq[r][c]) return;
    g_inq[r][c] = 1;
    g_qr[g_qtail] = r; g_qc[g_qtail] = c;
    g_qtail = (g_qtail + 1) % QMAX;
}
```
Without `g_inq`, a cell receiving grains from 4 neighbours could be enqueued 4 times and topple 4 times erroneously. The circular queue fits in `QMAX = MAX_ROWS × MAX_COLS + 1` slots.

**Two avalanche modes:**
- Instant (`full=1`): `while (g_qhead != g_qtail) { ... }` — drain to completion. Many drops per frame; watch the fractal grow.
- Vis (`full=0`): `break` after one topple. One topple per video frame — watch the cascade propagate cell by cell in red.

**Critical bug history:** An earlier `wave_end = g_qtail` snapshot caused the loop to exit after processing only the initial queue contents. Cells enqueued *during* toppling were abandoned — the cascade never propagated beyond one ring from centre. Fixed by removing the snapshot.

**Emergent fractal:** After millions of drops the stable pattern develops 4-fold quasi-crystalline symmetry. Avalanche size distribution follows a power law — the system self-tunes to criticality.

*Files: `fractal_random/sandpile.c`*

---

#### P14 SDF Metaballs — Smooth-Min Raymarching

`metaballs.c` renders 6 spheres whose SDFs are blended via polynomial smooth-min. The blended SDF creates a smooth organic surface that merges and separates as the balls orbit. Phong shading with soft shadows; surface curvature drives the color theme mapping.

**Scene SDF:**
```c
float sdf_scene(Vec3 p, ...) {
    float d = sdf_ball(p, centers[0], radii[0]);
    for (int i = 1; i < N_BALLS; i++)
        d = smin(d, sdf_ball(p, centers[i], radii[i]), k_blend);
    return d;
}
```
`smin(a, b, k) = min(a,b) − h²·k/4` where `h = max(k−|a−b|, 0)/k`. Small `k` ≈ hard min (separate spheres); large `k` ≈ wide merging neck (blob).

**Tetrahedron normal (4 SDF evaluations):** Samples SDF at 4 tetrahedron vertices `(±e,±e,±e)`. Gradient components: `nx = f0−f1−f2+f3`, `ny = −f0+f1−f2+f3`, `nz = −f0−f1+f2+f3`. Equivalent accuracy to 6-point central differences with 33% fewer evaluations.

**Curvature = Laplacian of SDF:** 7-point stencil, `CURV_EPS = 0.06`. At a sphere surface (r≈0.55): Laplacian ≈ 3.6 → `CURV_SCALE=0.25` → band 7 (warm). At a flat merged saddle: Laplacian ≈ 0 → band 0 (cool). Color shows the geometric structure even when the character shading is uniform.

**Soft shadows:** March a secondary ray toward the light; accumulate `min(sk·h/t)`. A ray grazing an occluder produces a small ratio → dark penumbra. `SHADOW_K = 8.0f` sets shadow softness.

**2×2 canvas block:** `CELL_W=2, CELL_H=2` renders each canvas pixel as a 2×2 terminal block. Reduces pixel count 4×. Aspect correction `phys_aspect = (h·2·2)/(w·2) = 2h/w` compensates for non-square terminal characters.

*Files: `raymarcher/metaballs.c`*

---

---

#### P12 De Bruijn Pentagrid — Penrose Tiling

`penrose.c` uses de Bruijn's algebraic duality to identify which Penrose rhombus contains each terminal cell in O(1), without storing any tiles.

**Five families of parallel lines:** Family j has direction `e_j = (cos(2πj/5), sin(2πj/5))`. For point (wx,wy) in pentagrid space: `k_j = floor(wx·cos_j + wy·sin_j)`. The 5-tuple (k_0…k_4) uniquely identifies which rhombus contains the point.

**Tile type from parity:** `S = Σ k_j`. Even S → thick rhombus (72°). Odd S → thin rhombus (36°). Crossing any grid line flips S parity, so thick and thin always alternate across every edge.

**Edge detection:** `frac(proj_j) ∈ [0,1)`. Distance to grid line = `min(frac, 1−frac)`. If the minimum across all five families is below BORDER=0.15, draw a directional line char whose angle = `2π·j/5 + π/2 − view_angle`.

**Color hash:** `h = abs(k_0·3 + k_1·7 + k_2·11 + k_3·13 + k_4·17) % 3`. Since adjacent tiles are always opposite type, same-type color collisions never cause invisible boundaries.

*Files: `fractal_random/penrose.c`*

---

#### P13 Diamond-Square Heightmap + Thermal Erosion

`terrain.c` builds a fractal landscape via midpoint displacement, then erodes it with thermal weathering.

**Diamond-square:** Alternate diamond step (square centres) and square step (edge midpoints). Each step adds `rand()*scale`; `scale *= ROUGHNESS=0.60` per level. Result: statistically self-similar heightmap where high-frequency roughness is proportional to elevation change.

**Thermal weathering:** `diff = h[y][x] − h[ny][nx]`. If `diff > TALUS`, move `EROSION_RATE*(diff−TALUS)` from high to low cell. Simulates granular material sliding when slope exceeds angle-of-repose. Run multiple passes per tick; mountains slowly smooth to plains.

**Bilinear interpolation** maps the fixed 65×65 grid to any terminal size without blocky nearest-neighbor artifacts.

*Files: `fractal_random/terrain.c`*

---

### Q — Artistic / Signal-Based Effects

#### Q1 Sinusoidal Aurora Curtains

Two-octave sinusoidal curtains with a vertical sine envelope render the northern-lights effect in `aurora.c`.

**Formula:** `primary = sin(x·1.5+t·0.2)·cos(y·3+t·0.5+x·0.25)`, `shimmer = cos(x·2.3−t·0.15)·sin(y·5+t·0.8+x·0.4)`, `v = primary·0.6 + shimmer·0.4`. Multiplied by `env = sin(π·y)` to suppress curtain at top and bottom edges. Two octaves break the regularity and create natural shimmer.

**Deterministic star hash:** `h = col·1234597 ^ row·987659; h ^= h>>13; h *= 0x5bd1e995; h ^= h>>15`. Zero storage, no flicker.

*Files: `artistic/aurora.c`*

---

#### Q2 Demoscene Plasma — Sin-Sum Palette Cycling

`plasma.c` sums four sinusoids per cell (horizontal, vertical, diagonal, radial) and maps through a cycling palette. Adding `fmod(time·CYCLE_HZ, 1.0)` to the normalized value before palette lookup shifts colors without changing the spatial pattern — zero physics.

*Files: `artistic/plasma.c`*

---

#### Q3 Hypotrochoid Spirograph with Float Canvas Decay

`spirograph.c` traces `x=(R−r)·cos(t)+d·cos((R−r)/r·t)`, `y=(R−r)·sin(t)−d·sin((R−r)/r·t)` onto a float canvas that decays by `FADE=0.985` per tick. No ring buffer needed — the canvas itself is the trail. Parameter drift slowly changes the petal count.

*Files: `geometry/spirograph.c`*

---

#### Q4 Voronoi with Langevin Brownian Seeds

`voronoi.c` moves seeds via `v += (−DAMP·v + NOISE·ξ)·dt` (Langevin dynamics). Per-cell nearest-seed search tracks d1 and d2; `d2−d1 < BORDER_PX` identifies Voronoi edges without Fortune's algorithm.

*Files: `geometry/voronoi.c`*

---

### R — Force-Based Physics

#### R1 Explicit Spring Forces + Symplectic Euler (Cloth)

**Why Jakobsen fails with pins:** Sequential constraint projection propagates pin corrections through the entire cloth, eliminating deformation. Force-based integration cannot be "corrected away."

**Spring force:** `F = k·stretch + kd·v_rel` where `v_rel = dot(v_b − v_a, n)`. Symplectic Euler: `v += a·dt; v *= DAMP; x += v·dt`.

*Files: `physics/cloth.c`*

---

#### R2 Softened N-Body Gravity + Verlet

**Softened gravity:** `F = G·m_i·m_j / (r² + ε²)^(3/2)`. ε² prevents singularities at close approach. O(n²) force loop, n=20 bodies, Velocity Verlet integration.

*Files: `physics/nbody.c`*

---

#### R3 Lorenz Strange Attractor — RK4 + 3-D Projection

**RK4** keeps the trajectory on the attractor without time-step reduction (Euler diverges due to chaos sensitivity). **Ghost trajectory** starts at x+ε=0.001 offset; exponential divergence on the Lyapunov time-scale (~1.1 s) is rendered in a contrasting color. **Orthographic rotation** `px = x·cos(a) − y·sin(a)` reveals 3-D structure without a projection matrix.

*Files: `physics/lorenz.c`*

---

---

### S — Barnes-Hut & Galaxy Simulation

#### S1 Barnes-Hut Quadtree — O(N log N) Gravity

**The problem:** Computing gravity naively requires O(N²) force pairs — every body pulls on every other. For N=400 that's 160,000 evaluations per tick; for N=10,000 it's 100 million.

**The key insight:** A group of bodies that is small and far away produces nearly the same gravitational effect as a single point mass located at their centre of mass. The Barnes-Hut algorithm quantifies "small and far" with the **opening angle criterion**:

```
s / d < θ   →  treat the group as one point mass
s / d ≥ θ   →  recurse into children
```

where `s` = side length of the quadtree cell, `d` = distance to the cell's centre of mass, and `θ ≈ 0.5` is the accuracy threshold.

**Algorithm:**
1. Build a quadtree over all bodies (O(N log N))
2. For each body i, traverse the tree: at each node, apply the opening angle test to decide whether to approximate or recurse
3. Integrate velocities and positions

The quadtree is rebuilt from scratch every tick using a static pool (see S3). This is cheaper than incremental updates because rebuilding is O(N log N) pool-writes and incremental rebalancing would require complex bookkeeping.

```c
/* Force traversal */
void qt_force(int ni, int bi, float *fx, float *fy) {
    QNode *n = &pool[ni];
    if (n->body_idx == bi) return;          /* skip self at leaf */

    float s = n->x1 - n->x0;               /* cell side */
    float d = dist(n->cx, n->cy, bodies[bi].px, bodies[bi].py);

    if (s/d < THETA || is_leaf(n)) {
        /* Single point-mass approximation */
        apply_gravity(bi, n->total_mass, n->cx, n->cy, fx, fy);
    } else {
        for (int c = 0; c < 4; c++)
            qt_force(n->child[c], bi, fx, fy);
    }
}
```

**Complexity:** O(N log N) average case. Each body visits O(log N) nodes in the well-separated regime. The actual cost depends on θ and on how clustered the bodies are.

*Files: `physics/barnes_hut.c`, `physics/nbody.c` (O(N²) for comparison)*

---

#### S2 Incremental Centre-of-Mass Update

When inserting body `b` into a tree node that already holds mass M at centre (cx, cy):

```c
float new_mass = n->total_mass + b->mass;
n->cx = (n->cx * n->total_mass + b->px * b->mass) / new_mass;
n->cy = (n->cy * n->total_mass + b->py * b->mass) / new_mass;
n->total_mass = new_mass;
```

This is the **weighted average** formula applied incrementally. It runs on every node from root to leaf during insertion. After all N bodies are inserted, every internal node's `(cx, cy)` correctly reflects the centre of mass of all bodies in its subtree — computed in a single pass with no post-processing.

**Why this is exact:** The two-body weighted average `(m₁·x₁ + m₂·x₂)/(m₁+m₂)` is algebraically equivalent to the full sum `Σmᵢxᵢ / Σmᵢ` when applied cumulatively. The incremental form handles arbitrary insertion order.

*Files: `physics/barnes_hut.c`*

---

#### S3 Static Node Pool — No malloc

```c
static QNode g_pool[NODE_POOL_MAX];   /* 16,000 nodes */
static int   g_pool_top = 0;

static int qt_alloc(...) {
    return g_pool_top++;              /* O(1) allocation */
}

/* Reset: */
g_pool_top = 0;                       /* O(1) — "frees" everything */
```

The pool is reset by setting `g_pool_top = 0`. All previous nodes are logically freed in a single write. This is safe because the entire tree is rebuilt from scratch each tick — no node persists across ticks.

**Capacity bound:** Each body insertion creates at most 4 new nodes (when splitting a leaf that held one body into 4 children and re-inserting both). For N=800: worst case ≈ 800×4 = 3200 nodes, well within the 16,000 cap.

**Why not malloc:** A single `malloc(sizeof(QNode))` per node at 60 Hz × 16,000 nodes/tick = 960,000 allocations/second. The heap overhead (lock contention, fragmentation, bookkeeping) would dominate the simulation cost. The static pool has zero overhead.

*Files: `physics/barnes_hut.c`*

---

#### S4 Keplerian Orbit Initialization

For a body orbiting a dominant central mass M_c, the circular orbit speed at radius r is:

```
v_kep = sqrt(G · M_c / r)
```

This comes from equating centripetal acceleration `v²/r` to gravitational acceleration `G·M_c/r²`.

In the galaxy preset, body 0 is a massive anchor (mass = N×3). All other bodies are placed at random radii and given Keplerian tangential velocity:

```c
float v_kep = sqrtf(g_G * M_bh / r);
float nx = -(py - cy) / r;   /* tangential unit vector (CCW) */
float ny =  (px - cx) / r;
body.vx = nx * v_kep;
body.vy = ny * v_kep;
```

**Differential rotation:** Inner bodies (small r) have shorter orbital period T = 2πr/v_kep = 2πr/√(GM/r) = 2π√(r³/GM). At r=0.1R, T is 1/√1000 ≈ 3% of the outer orbit period. This differential means inner bodies lap outer bodies, naturally winding any initial spiral pattern over time.

**Small eccentricity scatter:** A ±6% random perturbation to v_kep makes orbits slightly elliptical. This prevents the disk from looking artificially rigid and produces the precessing ellipse patterns characteristic of real stellar disks.

*Files: `physics/barnes_hut.c`, `artistic/galaxy.c`*

---

#### S5 Brightness Accumulator + Glow Decay

A float grid `g_bright[ROWS][COLS]` accumulates brightness from bodies and decays exponentially each render frame:

```c
/* Accumulate: each body increments its cell */
g_bright[row][col] += 1.0f;

/* Decay: every render frame */
g_bright[r][c] *= DECAY;   /* DECAY = 0.93 */

/* Steady-state: cell with f bodies/frame reaches B_ss = f/(1-DECAY) = 14.3 */
```

The max brightness at any cell is tracked each frame for normalisation: `norm = b / b_max`. This means the brightest cell always renders as the top character regardless of absolute body count — the ramp auto-scales to the current density.

**Decay timescale:** `DECAY=0.93` → half-life ≈ `ln(0.5)/ln(0.93) ≈ 9.5` render frames. At 30fps, a single body visit fades over ~0.3 seconds. Orbital trails (bodies visiting a cell repeatedly) maintain steady brightness.

**Difference from galaxy.c:** `galaxy.c` uses `DECAY=0.82` (half-life ~3.5 frames) because it has 3000 stars visiting cells densely and needs faster fade to prevent saturation. `barnes_hut.c` uses `DECAY=0.93` because bodies are sparse and slow decay keeps orbital paths visible.

*Files: `physics/barnes_hut.c`, `artistic/galaxy.c`*

---

#### S6 Speed-Normalized Color Mapping

To make body velocities visible, colors are mapped from speed relative to the rolling maximum:

```c
static float g_v_max = 1.0f;

/* After each tick: */
float spd = hypotf(body.vx, body.vy);
if (spd > g_v_max) g_v_max = spd;
g_v_max *= 0.9995f;   /* slow decay so colormap adapts */

/* In draw: */
float norm = spd / g_v_max;
int pair = (norm > 0.80f) ? CP_L5 :   /* blazing */
           (norm > 0.55f) ? CP_L4 :
           (norm > 0.30f) ? CP_L3 :
           (norm > 0.10f) ? CP_L2 : CP_L1;  /* nearly still */
```

The rolling max with slow decay (`×0.9995` per tick = −3% per second) ensures the full color range is used even when dynamics slow down. Without decay, a single high-speed ejection early in the simulation would fix g_v_max high for the entire run, making all subsequent bodies appear dim.

*Files: `physics/barnes_hut.c`*

---

### T — Wave Physics & Signal Analysis

#### T1 Analytic Wave Interference — Phase Precomputation

**Problem:** Computing `Σ A·cos(k·rᵢⱼ − ωt)` per cell per frame is expensive if `k·rᵢⱼ` is recomputed each frame.

**Solution:** Precompute `g_phase[s][row][col] = k · dist(source_s, cell)` once at startup (or on source move). Each frame only evaluates `cos(g_phase[s][r][c] − ω·t)`:

```c
/* Precompute (once per source position change) */
for each source s:
    for each cell (r,c):
        float dx = (c * CELL_W) - source[s].px;
        float dy = (r * CELL_H) - source[s].py;
        g_phase[s][r][c] = K * sqrtf(dx*dx + dy*dy);

/* Per frame (fast) */
float sum = 0;
for each source s:
    sum += cosf(g_phase[s][r][c] - omega * t);
sum /= n_sources;   /* normalise to [-1,+1] */
```

The per-cell cost drops from `sqrt + mul + cos` to just `cos + sub` per source per frame.

**Signed 8-level ramp:** `sum ∈ [-1,+1]` maps to 8 color pairs: 3 negative (blue), 1 neutral, 3 positive (red), 1 white peak. This directly visualises the wave physics — destructive interference = blue, constructive = red/white.

*Files: `fluid/wave_interference.c`*

---

#### T2 Lyapunov Exponent — Alternating Logistic Map

The **logistic map** `xₙ₊₁ = r·xₙ·(1−xₙ)` transitions from stable fixed points (r < 3) to period doubling (3 < r < 3.57) to chaos (r > 3.57) as r increases.

The **Lyapunov exponent** `λ = (1/N) Σ ln|f'(xₙ)| = (1/N) Σ ln|r(1−2xₙ)|` measures the average rate of divergence of nearby trajectories:
- λ < 0: trajectories converge → **stable** (fixed point or cycle)
- λ = 0: bifurcation boundary
- λ > 0: trajectories diverge → **chaos**

The **Lyapunov fractal** assigns each pixel `(a, b)` as a parameter set: the logistic map alternates between rate `a` and rate `b` in the sequence AABBA or ABABAB (specified per preset). Each pixel computes λ for its (a,b) pair.

```c
/* For pixel at (a, b): */
float x = 0.5f;
float lambda = 0.0f;
for (int n = 0; n < ITER; n++) {
    float r = (seq[n % seq_len] == 'A') ? a : b;
    x = r * x * (1.0f - x);
    float deriv = fabsf(r * (1.0f - 2.0f*x));
    if (deriv > 0) lambda += logf(deriv);
}
lambda /= ITER;
```

**int8_t bucket encoding:** The computed λ is stored as a signed 8-bit integer (×32 scale) rather than a float, saving 4× memory and enabling fast palette lookup via sign bit.

*Files: `fractal_random/lyapunov.c`*

---

#### T3 DBM Laplace Growth — Gauss-Seidel + φ^η

**Dielectric Breakdown Model (DBM):** Models lightning, dendritic crystal growth, and Laplacian fractal growth. A voltage field `φ` satisfies Laplace's equation `∇²φ = 0` (Gauss-Seidel iteration). Growth probability at each frontier cell is proportional to `φ^η` — higher η produces thinner, more branchy structures (η=1: DLA-like, η=∞: straight line to the electrode).

**Algorithm:**
1. Set `φ=1` at top row (electrode), `φ=0` at aggregate cells (conductor)
2. Iterate Gauss-Seidel: `φ[r][c] = 0.25·(φ[r-1][c] + φ[r+1][c] + φ[r][c-1] + φ[r][c+1])` for all non-boundary, non-aggregate cells
3. Find all frontier cells (empty cells adjacent to aggregate)
4. Select one frontier cell with probability ∝ `φ[r][c]^η`
5. Add it to aggregate; set `φ[r][c] = 0`
6. Repeat

The key difference from pure DLA (random walker) is that the Laplace field `φ` is computed explicitly — growth follows the electric field gradient rather than random diffusion, giving sharper branching patterns.

**Neumann boundary conditions:** Side walls get `∂φ/∂n = 0` (no flux): `φ[r][0] = φ[r][1]`, `φ[r][cols-1] = φ[r][cols-2]`. This lets the field bend naturally around obstacles without artificial reflection.

*Files: `fractal_random/tree_la.c`*

---

### P15 — Harmonograph / Lissajous (`geometry/lissajous.c`)

`lissajous.c` draws two perpendicular damped oscillators whose phase drifts slowly, morphing the parametric figure through figure-8s, trefoils, stars, and spirals.

**Equations:** `x(t) = sin(fx·t + phase)·exp(−decay·t)`, `y(t) = sin(fy·t)·exp(−decay·t)`. `T_MAX = N_LOOPS·2π/min(fx,fy)` so every ratio runs exactly 4 cycles of the slower oscillator. `DECAY = DECAY_TOTAL/T_MAX` — always 1% amplitude at T_MAX.

**Age rendering:** Draws oldest inner loops (dim `.`) first, newest outer loops (bright `#`) last. Newest overwrites shared cells. `age = i/(N-1)` where 0=newest → level 0, 1=oldest → level 3.

**Phase dwell:** `key_period = π/max(fx,fy)`. Near each symmetric phase, drift is multiplied to 0.25× and ramps linearly back to 1× over DWELL_WIDTH=0.25 of the period. After 2π phase sweep the ratio auto-advances.

**8 frequency ratios:** 1:2 Figure-8, 2:3 Trefoil, 3:4 Star, 1:3 Clover, 3:5 Pentagram, 2:5 Five-petal, 4:5 Crown, 1:4 Eye.

**Keys:** `space` pause, `n`/`p` next/prev ratio, `+`/`-` drift speed, `c` theme, `q` quit.

**Build:** `gcc -std=c11 -O2 -Wall -Wextra geometry/lissajous.c -o lissajous -lncurses -lm`

*Files: `geometry/lissajous.c`*

---

---

### I — Raymarching & SDF (new entries)

#### I5 Mandelbulb Distance Estimator
The Mandelbulb extends Mandelbrot iteration to 3D using spherical power: `z^p → r^p · (sin(pθ)cos(pφ), sin(pθ)sin(pφ), cos(pθ))` then `z += c`. The distance estimator tracks the derivative magnitude `dr = p · r^(p-1) · dr + 1` alongside the iteration, giving `DE = 0.5 · log(r) · r / dr`. When `dr` is large the surface is far; when small the march is close. Power 8 gives the classic 8-fold symmetric Mandelbulb; powers 2–12 sweep from sphere to full fractal.
*Files: `raymarcher/mandelbulb_explorer.c`, `raster/mandelbulb_raster.c`*

#### I6 Smooth Escape-time Coloring
Integer escape count `i` produces harsh banding where adjacent iteration shells have discrete color jumps. The continuous formula: `mu = i + 1 − log(log(|z|)/log(bail)) / log(power)`. The `log(log(|z|))` term measures how far past the bailout the orbit was, interpolating continuously between integer counts. `mu ∈ ℝ` produces smooth color gradients across depth shells.
*Files: `raymarcher/mandelbulb_explorer.c`*

#### I7 Orbit Trap Coloring
During Mandelbulb iteration, track the minimum distance from any orbit point to a geometric object: `trap = min(trap, |z.y|)` (distance to XY plane). Points where the orbit stayed close to the trap get a different hue from points where it diverged widely. Orbit traps reveal the internal structure of the attractor basin, creating the characteristic "tentacle" and "pod" coloring visible on the surface.
*Files: `raymarcher/mandelbulb_explorer.c`*

#### I8 Near-miss Glow via min_d Tracking
During sphere marching, track the minimum DE value ever reached: `min_d = min(min_d, d)`. When the ray misses but came close (`min_d < GLOW_RANGE`), `glow_str = (1 − min_d/GLOW_RANGE)^3`. Pixels with glow_str > 0 receive a dim edge glow character — a halo around the fractal silhouette. This requires no extra DE calls; the march loop records min_d for free.
*Files: `raymarcher/mandelbulb_explorer.c`*

#### I9 Progressive ROWS_PER_TICK Rendering
Raymarching the full screen at 60 fps is too slow for complex fractals. Progressive rendering processes `ROWS_PER_TICK=4` rows per frame, maintaining UI responsiveness. A `g_stable` buffer holds the last complete frame, displayed while the new scan sweeps from top to bottom. `g_dirty=true` (set on user input) resets the scan row; morph mode avoids setting dirty so the radar-sweep scan becomes visible as an animation effect.
*Files: `raymarcher/mandelbulb_explorer.c`, `raymarcher/sdf_gallery.c`*

#### I10 SDF Boolean Operations — Union, Intersection, Subtraction
Three operations combine any two SDFs `a` and `b` into compound geometry:
- **Union** `min(a, b)` — the closest surface; merges two objects into one.
- **Intersection** `max(a, b)` — only the region where both overlap; carves away everything outside both.
- **Subtraction** `max(a, -b)` — A minus B; drills B-shaped holes into A.

These operations compose freely — any SDF expression can replace `a` or `b`, enabling arbitrarily complex Boolean trees with zero mesh bookkeeping.
*Files: `raymarcher/sdf_gallery.c`*

#### I11 Smooth Union — Polynomial smin Blend
Hard `min(a,b)` creates a sharp crease where two surfaces meet, visible as a crisp edge even at terminal resolution. The polynomial smooth minimum `smin(a, b, k) = min(a,b) − h²·k/4` where `h = max(k − |a−b|, 0) / k` blends the two fields within a transition band of width `k`. At the blend boundary both fields are pulled toward each other, creating a rounded neck or merge region. `k = 0` recovers hard min; `k = 0.1` is a subtle join; `k = 1.0` produces a bulgy organic merge. Used in scene5_sculpt to assemble an organic figure: the SDF for each body part is a simple primitive, smin welds them into continuous skin.
*Files: `raymarcher/sdf_gallery.c`*

#### I12 Twist Deformation — Pre-warp Before SDF Evaluation
Applying a rotation to the query point `p` before evaluating an SDF deforms the shape in world space. Twist rotates the xz-plane by an angle proportional to height: `p.xz = rot2(p.y · k) · p.xz`. The same box SDF evaluated on this twisted `p` produces a twisted box. Important caveat: the twist operation stretches the metric — distances become locally inaccurate by a factor proportional to the twist rate. The march step must be made conservative (e.g. `0.60×` instead of `0.85×`) to avoid stepping through thin geometry.
*Files: `raymarcher/sdf_gallery.c`*

#### I13 Domain Repetition — Infinite Lattice from One Primitive
`p_rep = p − cell · round(p / cell)` maps any query point to the nearest cell center, effectively tiling the primitive at period `cell` in that axis. This costs one SDF evaluation regardless of how many copies are visible. The lighting and shadow rays must use the original (unrepeated) `p` for light direction; repeating the light direction as well would give different shadows per copy. Only the SDF evaluation inside `sdf_march` sees the repeated `p`.
*Files: `raymarcher/sdf_gallery.c`*

---

### J — Software Rasterization (new entries)

#### J18 Sphere Projection Tessellation (Implicit Surface → Mesh)
For surfaces with no analytic parametrisation (Mandelbulb, metaballs), tessellate by projecting inward from a UV sphere: for each `(θ, φ)` direction, march from `r=1.5` inward until `DE < HIT_EPS`. Record the first hit point as a mesh vertex with position, SDF-gradient normal, and smooth iteration value. Connect valid neighboring vertices into quads → triangles. This captures the outer surface skin only — concavities and interior structure are invisible. The trade-off vs raymarching: faster per-frame render, fewer visual features.
*Files: `raster/mandelbulb_raster.c`*

#### J19 rgb_to_cell — Full-color Framebuffer via 216-color Cube
`luma_to_cell` (cube_raster.c) maps luminance → one of 7 fixed color pairs, losing hue information. `rgb_to_cell` initialises all 216 xterm color pairs (16+r×36+g×6+b, r/g/b ∈ 0..5) and maps fragment RGB to the nearest cube entry. The Bourke density ramp still controls the character from luminance; the color pair now faithfully represents the fragment's actual hue. Required for normals shader (6-hue visualisation), hue-depth shader, and any fragment output with meaningful color content.
*Files: `raster/mandelbulb_raster.c`*

#### J20 HSV → RGB for Fragment Shaders
`hsv_to_rgb(h, s, v)` converts a hue angle `h∈[0,1)` to RGB. `h × 6` selects one of 6 hue sectors; fractional part `f` interpolates within the sector. `p = v(1−s)`, `q = v(1−sf)`, `t = v(1−s(1−f))` are the three mixing values. Used to create smooth rainbow gradients from scalar values (escape time → hue, normal azimuth → hue, depth → hue). Placing this in §2 math keeps it available to all three fragment shaders.
*Files: `raster/mandelbulb_raster.c`*

---

### K — Shading Models (new entries)

#### K5 3-Point Lighting Setup — Key + Fill + Rim
Standard film/game lighting rig with three independent lights:
- **Key** (strong, warm, animated sweep): main source of diffuse + specular. `0.65 × diffuse + 0.55 × specular`
- **Fill** (weak, cool, opposite side): prevents the shadow side from being pure black. `0.22 × diffuse` only.
- **Rim** (narrow, from behind): creates a bright edge on the silhouette, separating object from background. `0.18 × diffuse + 0.65 × specular` with low shininess (exponent 10) for broad rim highlight.
Sum = `ambient + key_diffuse + key_spec + fill_diffuse + rim_diffuse + rim_spec`. Each light uses its own color to enable warm/cool contrast.
*Files: `raymarcher/mandelbulb_explorer.c`, `raymarcher/sdf_gallery.c`*

#### K6 Depth Visualization Shader — View-space Depth → Color
The vertex shader computes view-space depth: `custom[0] = length(world_pos − cam_pos)`. The fragment shader normalizes to `[cam_dist − 1.5, cam_dist + 1.5]` and maps `depth_t ∈ [0,1]` to a color gradient (near=warm, far=cool). Combined with the Bourke ramp for characters, this creates a depth fog effect that reveals the 3D structure of any mesh without lighting computation. Useful for debugging mesh geometry.
*Files: `raster/mandelbulb_raster.c`*

#### K7 Three Lighting Modes — N·V / Phong / Flat
A single integer `light_mode` switches between three shading strategies, cycled at runtime with the `l` key:

- **N·V (`light_mode=0`)**: `luma = KA + KD·dot(N,V)·ao`. Brightness = how directly the surface faces the camera. No global light direction means no competing brightness gradient — the material color (theme `col` field) becomes the dominant visual signal. Best for understanding shape.
- **Phong (`light_mode=1`)**: Full 3-point rig with shadow and AO. Best for cinematic presentation and depth reading.
- **Flat (`light_mode=2`)**: Returns `1.0` immediately. Every hit pixel gets the densest Bourke char; only the theme hue varies. No shadow or AO computation — fastest render. Useful for inspecting the color distribution across a scene without lighting distraction.

The implementation short-circuits early:
```c
if (light_mode == 2) return 1.0f;               /* Flat: no computation */
if (light_mode == 0) { return KA + KD*ndv*ao; } /* N·V: no light direction */
/* else: full Phong */
```
*Files: `raymarcher/sdf_gallery.c`*

---

### U — Path Tracing & Global Illumination

#### U1 Monte Carlo Path Tracing
A **path tracer** solves the rendering equation `L_o = L_e + ∫ L_i · f_r · cosθ dω` by Monte Carlo sampling: fire one ray per pixel, bounce it randomly, accumulate radiance from emissive hits. The estimate is unbiased — more samples converge to the exact ground-truth image. Key phenomena that emerge automatically: soft shadows (area light sampling), color bleeding (diffuse inter-reflection), ambient occlusion (geometry occlusion), caustics (specular → diffuse paths).
*Files: `raytracing/path_tracer.c`*

#### U2 Lambertian BRDF + Cosine Hemisphere Sampling
For Lambertian BRDF `f_r = ρ/π` and cosine-weighted hemisphere PDF `p(ω) = cosθ/π`, the Monte Carlo weight simplifies: `f_r · cosθ / p = (ρ/π) · cosθ / (cosθ/π) = ρ`. The π and cosθ cancel — `throughput *= albedo` is the complete update. Malley's method generates cosine-weighted samples: uniform disk sample `(r1,r2)` → azimuth `φ=2πr1`, elevation `sinθ=√r2`, `cosθ=√(1−r2)`, transform to world space via ONB around N.
*Files: `raytracing/path_tracer.c`*

#### U3 Russian Roulette — Unbiased Path Termination
Fixed-depth truncation misses deep indirect light (biased). Russian roulette terminates at any depth `d ≥ RR_DEPTH` with probability `1 − p` where `p = max(throughput)`. Surviving paths compensate with `throughput /= p`. Unbiased proof: `E[contribution] = (contribution × p) / p = contribution`. Dim paths (near-zero throughput, small p) are more likely to die — exactly correct since they contribute negligibly even if they survive.
*Files: `raytracing/path_tracer.c`*

#### U4 Progressive Accumulator — Convergent Rendering
`g_accum[y][x][3]` stores the running sum of radiance samples. Each frame adds `SPP_PER_FRAME` jittered samples (sub-pixel random offset = free anti-aliasing). Display = `accum / samples` after tone-mapping. The image evolves from white noise → recognizable structure (16 samples) → converged (512+ samples). Reset on resize (`memset+0`) or user request. Auto-stop at `ACCUM_CAP=8192` since convergence past that is imperceptible at ASCII resolution.
*Files: `raytracing/path_tracer.c`*

#### U5 Reinhard Tone Mapping
Raw path-traced radiance is unbounded (`[0, ∞)`). Reinhard compresses per channel: `L_display = L / (1 + L)`. Properties: passes 0→0, maps 1→0.5, asymptotes to 1 as L→∞. Follow with gamma encode `L^(1/2.2)` for perceptual sRGB. Filmic alternatives (Hejl-Dawson, ACES) give more dramatic contrast but Reinhard is sufficient for terminal resolution. The light emission of 15 W·sr⁻¹ compresses to `15/16 ≈ 0.94` — near-white as intended.
*Files: `raytracing/path_tracer.c`*

#### U6 xorshift32 — Decorrelated Per-pixel RNG
```c
static Rng rng_seed(int px, int py, int frame) {
    uint32_t s = px*1973 + py*9277 + frame*26699 + 1;
    s ^= s<<13; s ^= s>>7; s ^= s<<17;
    return s ? s : 1u;
}
```
Independent per-pixel seeds prevent correlated noise (bands, streaks). The hash mixes pixel coordinates and frame index — adjacent pixels and adjacent frames get unrelated initial states. Three warm-up xorshift steps break up any regularities in the hash output. The `s ? s : 1` guard prevents the all-zero state (xorshift(0) = 0 forever).
*Files: `raytracing/path_tracer.c`*

#### U7 Cornell Box — Standard Path Tracing Scene
The Cornell Box is the benchmark for global illumination algorithms: red left wall, green right wall, white floor/ceiling/back, small overhead area light. Designed to show: color bleeding (red/green walls tint white surfaces), soft shadows (finite area light), and indirect illumination (ceiling lit by floor reflections). Two spheres with distinct colors add inter-object color bleeding. Any path tracer that renders the Cornell Box correctly handles all first-order global illumination effects.
*Files: `raytracing/path_tracer.c`*

#### U8 Axis-aligned Quad Intersection
```c
t = (pos_axis − ray.o[axis]) / ray.d[axis]
hit.u = ray.o[u_axis] + t × ray.d[u_axis]   check ∈ [lo[0], hi[0]]
hit.v = ray.o[v_axis] + t × ray.d[v_axis]   check ∈ [lo[1], hi[1]]
```
One-dimensional solve on the fixed axis, then two bound checks. Normal is always flipped to face the incoming ray: if `rd[axis] > 0` → normal = `-axis_dir`. This guarantees hemisphere sampling is always away from the surface, regardless of which side the ray enters from. The light quad at `y=0.98` tests before the ceiling at `y=1.0` (smaller t) so rays see the light first in its footprint.
*Files: `raytracing/path_tracer.c`*

---

---

### D — Physics Simulation (new entries)

#### D9 Impulse-based Elastic Collision Resolution

Two discs are approaching when the dot product of their relative velocity along the collision normal is positive: `vn = dot(va - vb, n) > 0`. Applying an impulse `J = (1 + e) · vn / (1/ma + 1/mb)` along the normal simultaneously adjusts both velocities while conserving momentum and kinetic energy. The restitution coefficient `e = 1` gives perfectly elastic collisions; `e < 1` dissipates energy.

Penetration is a separate problem from velocity. Before resolving velocity, separate the overlapping bodies by pushing each along the normal by `overlap/2 / (1/ma + 1/mb)`. Doing this separation first and then resolving velocity — rather than encoding penetration correction inside the velocity update — means the impulse only fires when the bodies are genuinely approaching, preventing false bounces on already-separating pairs.

Equal-mass collisions along the normal reduce to swapping the normal velocity components exactly. Non-equal masses cause the lighter body to deflect more — matching billiard-ball intuition where a cue ball deflects when striking a heavier ball.
*Files: `physics/elastic_collision.c`*

---

#### D10 Lorentz Force — Exact Rotation Integration

A charged particle in a uniform magnetic field perpendicular to the screen experiences the Lorentz force `F = q v × B`, which continuously deflects the velocity perpendicular to itself — producing circular orbits. The cyclotron radius is `r = m|v| / (qB)`: high charge-to-mass ratio (electrons) → tight spirals; low ratio (protons) → gentle arcs.

Naive Euler integration (`v += F·dt`) computes the force using the velocity at the start of the step, then updates. This introduces a systematic error that inflates the orbit radius each step — the particle spirals outward rather than maintaining a circle. The fix is to apply an exact 2D rotation matrix `R(ω·dt)` to the velocity each step, where `ω = (q/m)·B`:

```c
float c = cosf(omega * dt);
float s = sinf(omega * dt);
float vx_new = vx * c - vy * s;
float vy_new = vx * s + vy * c;
```

This preserves `|v|` exactly. Ionisation energy loss (the drag that makes bubble-chamber tracks spiral inward) is added afterward as a multiplicative speed reduction: `|v| *= (1 - DRAG)`. The combined effect — exact rotation plus shrinking speed — produces the logarithmic inward spiral that characterises real particle tracks.
*Files: `physics/bubble_chamber.c`*

---

#### D11 Euler Equations for Rigid-Body Rotation — Quaternion Tracking

A free rigid body with three unequal principal moments of inertia (`I1, I2, I3`) obeys Euler's equations in the body frame:

```
I1·ω̇x = (I2 - I3)·ωy·ωz
I2·ω̇y = (I3 - I1)·ωz·ωx
I3·ω̇z = (I1 - I2)·ωx·ωy
```

These are nonlinear (the right-hand side contains products of angular velocities), so RK4 integration is used to keep the trajectory on the attractor. A key result is the intermediate-axis theorem: rotation near the axis of smallest or largest inertia is stable, but rotation near the middle-inertia axis is unstable — a perturbation grows exponentially, producing the tumbling "Dzhanibekov effect."

Orientation is tracked as a unit quaternion `q = (qw, qx, qy, qz)`. The quaternion derivative is `q̇ = 0.5 · q ⊗ (0, ω)` where `(0, ω)` is a pure quaternion from the body-frame angular velocity. After each RK4 step the quaternion is re-normalised (`q /= |q|`) and the world-space rotation matrix is extracted from it. This avoids gimbal lock (which Euler angles suffer at `θ = 0`) and the accumulated drift that rotation-matrix integration has.
*Files: `physics/gyroscope.c`*

---

#### D12 Three-Body Choreography — Figure-8 Orbit

The three-body problem has no general closed-form solution. Specific initial conditions discovered numerically produce periodic "choreographies" — orbits where all three equal-mass bodies traverse the same closed curve at 120° phase offsets. The figure-8 solution (Chenciner and Montgomery, 2000) is the most famous: three bodies chase each other around a figure-8 path forever.

The initial conditions must be specified to high precision (five or more significant figures) because the orbit is only marginally stable — small perturbations cause the bodies to diverge on the Lyapunov timescale. RK4 with small `dt = 0.001` in natural units preserves the orbit for hundreds of visible periods; Velocity Verlet at the same step size drifts and eventually breaks the choreography.

The centre-of-mass frame correction — subtracting the mean velocity each step — keeps the simulation centred on screen. Without it, momentum imbalance from floating-point rounding causes the whole system to drift off screen over time. The `x` key in the implementation adds a random perturbation to one body, instantly revealing the underlying chaotic dynamics hiding beneath the symmetric orbit.
*Files: `physics/orbit_3body.c`*

---

#### D13 Pendulum Wave — Analytic Harmonic Superposition

N pendulums are assigned lengths such that pendulum `n` completes exactly `(N_BASE + n)` full oscillations in a fixed synchronisation period `T_SYNC`. The angular frequency is `ω_n = 2π(N_BASE + n) / T_SYNC` and position is the exact analytic solution `θ_n(t) = A · sin(ω_n · t)` — no numerical integration required. Each pendulum is an independent harmonic oscillator with a known exact solution.

Because consecutive pendulums differ by exactly one oscillation per `T_SYNC`, at `t = T_SYNC` all pendulums have completed an integer number of cycles and are perfectly back in phase — the "clap" resync. At intermediate times the superposition of N slightly-different frequencies creates travelling waves, standing waves, and apparent spirals depending on how close `t` is to simple fractions of `T_SYNC`.

The key rendering detail is drawing the string as a sloped line from the pivot to the bob using slope-appropriate characters (`|`, `/`, `\`), not as a vertical column. Drawing a vertical column makes the string appear to disappear while only the bob moves — the sloped line gives the correct visual impression of the pendulum's angle.
*Files: `physics/pendulum_wave.c`*

---

#### D14 Slider-Crank Kinematics — Engine Cycle Simulation

The slider-crank mechanism converts rotary crank motion to linear piston motion. Given crank angle `θ`, crank radius `R`, and connecting rod length `L`, the wrist-pin row in cell space is:

```
rod_vert = sqrt(L² - (R·sin(θ))²)
wrist_row = crank_centre_row - R·cos(θ) - rod_vert
```

This is exact geometry with no small-angle approximation. Port timing (when the exhaust and transfer ports open and close) is derived directly from the piston crown position rather than from a separate lookup table: `ex_open = (crown_row > engine_top + EX_PORT_OFF)`. This keeps timing numerically consistent with the drawn geometry at any engine speed.

A 2-stroke cycle completes its five phases in a single crankshaft revolution: compression → ignition (TDC spark) → power stroke → exhaust port opens (burned gas escapes) → transfer port opens (fresh mixture scavenges cylinder). Detecting the current phase from crank angle and port state drives character and color changes for the gas above the piston, exhaust flow, and transfer flow — an animated cross-section that teaches engine thermodynamics entirely through ASCII character choices.
*Files: `physics/2stroke.c`*

---

#### D15 Baumgarte Stabilised Rigid-Body Collision

Iterative impulse resolution with Baumgarte position correction is the industry-standard method for 2D/3D rigid-body engines. Each solver iteration consists of two passes for every overlapping pair:

**Pass A — positional correction (always):** Move overlapping bodies apart by a fraction of the penetration depth: `corr = max(depth - SLOP, 0) × BAUMGARTE / (imA + imB)`. The SLOP constant (≈0.05 px) allows a tiny tolerated penetration that absorbs floating-point noise at resting contacts without triggering correction every frame. BAUMGARTE (≈0.5) spreads the correction over multiple frames for stability.

**Pass B — velocity impulse (only when approaching):** Compute the normal relative velocity `vn = dot(va - vb, n)`. If `vn ≤ 0` the bodies are already separating — skip. Otherwise apply `j = (1 + e_eff)·vn / (imA + imB)`. Adaptive restitution: when `vn < REST_THRESH` (gravity-scale approach speed), set `e_eff = 0` to exactly cancel the incoming velocity instead of bouncing — this eliminates the floor-jitter bug where gravity generates infinite micro-bounces at rest.

Running `SOLVER_ITERS = 10` passes propagates contact corrections through stacked bodies (resolving the bottom pair propagates upward through the stack). A sleep system (freezing bodies whose speed stays below `SLEEP_VEL` for `SLEEP_FRAMES` frames) eliminates CPU cost for settled stacks.
*Files: `physics/rigid_body.c`*

---

#### D16 Quantum Wavepacket — Crank-Nicolson / Thomas Algorithm

The time-dependent Schrödinger equation `iℏ∂ψ/∂t = −(ℏ²/2m)∂²ψ/∂x² + V(x)ψ` is solved with the Crank-Nicolson finite-difference scheme. Averaging the explicit and implicit Euler steps produces a system `(I + iHdt/2)ψⁿ⁺¹ = (I − iHdt/2)ψⁿ` where `H` is the discrete Hamiltonian matrix. This scheme is unconditionally stable and exactly unitary — the L2 norm `Σ|ψ|²` is preserved to machine precision regardless of timestep size.

The resulting linear system is tridiagonal (three diagonals, because the Hamiltonian only couples each grid point to its two neighbours). The Thomas algorithm (forward elimination then back-substitution) solves it in O(N) operations rather than O(N³) for a general matrix. Initial state is a Gaussian wave packet `ψ₀(x) = exp(−((x−x₀)/σ)²/2) · exp(ik₀x)` — a particle localised at `x₀` with momentum `ℏk₀`.

Quantum tunnelling is visible when the packet encounters a potential barrier `V > E`. The transmitted amplitude is `T ≈ exp(−2κd)` where `κ = √(2m(V−E))/ℏ` and `d` is barrier width. Absorbing boundaries (multiplying `ψ` by a smooth damping function near the edges) suppress reflections from the grid walls that would otherwise contaminate the physics.
*Files: `physics/schrodinger.c`*

---

#### D17 Tripod Gait FSM — Timer + Landing Condition

A hexapod's six legs split into two groups that alternate stepping. The gait state machine advances a `phase_timer` each tick and transitions only when two conditions are both true: `phase_timer ≥ PHASE_DURATION` and every leg in the current stepping group has landed (`stepping[i] == false` for all three). The dual condition is essential: at low speeds a foot may still be swinging when the timer expires; at high speeds the timer may not expire before all feet land. Requiring both prevents mid-air group launches.

On transition, the new group's legs snapshot their current positions as `foot_old`, compute a new `step_target = rest_target(hip, i)` (hip offset plus forward lookahead), and set `stepping = true`. The step animation advances `step_t ∈ [0,1]` each tick at rate `dt / STEP_DURATION`. Horizontal progress uses `smoothstep(step_t)` for ease-in/ease-out; vertical arc uses `−STEP_HEIGHT × sin(π × step_t)` on the raw `step_t` for a symmetric bell curve.

*Files: `animation/hexpod_tripod.c`*

---

#### D18 Two-Joint Analytical IK — Law of Cosines

Given hip H and foot T with femur length U and tibia length L:

1. `dist = clamp(|T−H|, |U−L|+1, U+L−1)` — project to reachable shell
2. `base = atan2(dy, dx)` — polar angle from hip to foot
3. `cos_h = (dist² + U² − L²) / (2·dist·U)` — law of cosines for the hip angle
4. `ah = acos(clamp(cos_h, −1, 1))` — hip half-angle
5. Knee angle: `base − ah` (left legs, knee bends outward above body) or `base + ah` (right legs, knee bends outward below body)
6. `knee = hip + U × (cos(knee_angle), sin(knee_angle))`

The `clamp` before `acos` prevents NaN when floating-point rounding pushes `cos_h` outside `[−1, 1]`. The sign choice (subtract vs. add `ah`) controls which side of the hip-foot line the knee sits on; this must match the physical anatomy — legs on opposite sides of the body need opposite signs.

The key insight for rotation-invariant IK: the sign convention is fixed relative to the leg's side (left/right), not relative to world axes. For any body heading the left knee always bends in the correct outward direction because `base` already encodes the current hip-to-foot angle in world space.

*Files: `animation/hexpod_tripod.c`, `animation/ik_spider.c`*

---

#### D19 Angular Interpolation — Short-Arc Heading Steering

Directly adding `(target − current) × rate` to a heading angle fails when the angles cross the ±π branch cut: a 350° → 10° turn computes a −340° difference instead of the correct +20°. The fix is to normalise the difference to `[−π, π]` before clamping:

```c
float diff = target - current;
while (diff >  M_PI) diff -= 2*M_PI;
while (diff < -M_PI) diff += 2*M_PI;
float turn = clamp(diff, -RATE*dt, RATE*dt);
current += turn;
```

The `while` loops (not `if`) handle the rare case where multiple full revolutions accumulate. The clamped turn rate ensures the robot turns gradually regardless of how large the target change is — pressing the opposite arrow key produces a smooth U-turn rather than an instant snap. The same normalisation is needed when alpha-lerping heading in the render layer.

*Files: `animation/hexpod_tripod.c`*

---

### E — Cellular Automata & Grid Simulations (new entries)

#### E6 Hexagonal Grid CA — Offset-Row Neighbour Addressing

A hexagonal grid stored as a 2D rectangular array uses the offset-row convention: even rows align to column centres; odd rows are shifted half a cell to the right. This means the six neighbours of cell `(i, j)` depend on the row parity:

```c
/* Even row i */
int nbr_even[6][2] = {{-1,0},{-1,1},{0,-1},{0,1},{1,0},{1,1}};
/* Odd row i */
int nbr_odd[6][2]  = {{-1,-1},{-1,0},{0,-1},{0,1},{1,-1},{1,0}};
```

The rule bitmask is indexed by neighbour count `n ∈ {0..6}`: `(BIRTH_MASK >> n) & 1` for dead cells, `(SURVIVE_MASK >> n) & 1` for live cells. This compact representation makes it trivial to explore the 2⁷ × 2⁷ rule space. The B2/S34 rule produces gliders and stable oscillators with qualitatively different shapes than square-grid Life — the 6-neighbour topology is more isotropic, eliminating diagonal propagation artefacts. On screen, alternate rows are displayed offset by one column, so the character grid visually matches the honeycomb geometry.
*Files: `artistic/hex_life.c`*

---

#### E7 Lenia — Continuous CA with Ring Kernel

Lenia generalises Conway's Game of Life to continuous values, time, and space. Each cell holds a real value `u ∈ [0,1]`. The neighbourhood is a ring-shaped kernel `K` — a bump function `exp(4 - 4/(r(1-r)))` evaluated at normalised radius `r = dist/R`, which is zero inside and outside a thin annular shell. The update rule:

```
U(x,t) = K ⊛ A(·,t)                  ← convolution (sensing)
G(u)   = 2·exp(-((u-μ)/σ)²) - 1      ← growth function (Gaussian bell)
A(x,t+dt) = clamp(A + dt·G(U), 0, 1) ← state update
```

The growth function `G` reaches +1 when the local density matches the species parameter `μ` exactly, and falls to −1 when density is too low or too high. This "sweet-spot" mechanism produces self-organising creatures analogous to biological cells responding to chemical gradient cues.

Precomputing the kernel once as a flat list of (row-offset, col-offset, weight) triples amortises the O(R²) cost and enables cache-friendly inner loops. Different parameter pairs `(μ, σ, R)` define distinct "species": the Orbium (μ=0.15, σ=0.015, R=13) moves coherently like a glider; the Aquarium produces a rich soup of interacting blobs.
*Files: `fluid/lenia.c`*

---

#### E8 Greenberg-Hastings Excitable Media CA

The Greenberg-Hastings model is a three-state CA (resting → excited → refractory → resting) that produces spiral waves, target patterns, and travelling wave trains. A resting cell becomes excited if it has at least one excited neighbour. An excited cell immediately enters the refractory state. A refractory cell returns to resting after a fixed recovery time.

The FitzHugh-Nagumo variant used in `reaction_wave.c` extends this to continuous values with two coupled PDE fields: activator `u` (fast membrane voltage) and inhibitor `v` (slow recovery variable). The activator has a cubic nonlinearity `u³/3` that creates the excitation threshold; the inhibitor rises after each spike and suppresses re-excitation during the refractory period. Diffusion via the 5-point Laplacian stencil spreads the wavefront spatially.

Spiral waves require a broken wavefront for initiation: create a planar wave, then make a section of the medium refractory (blocking that half). The free end of the planar wave curls into a rotating spiral. Once established, spirals are self-sustaining — they do not require continuous external forcing, and two counter-rotating spirals can annihilate each other on contact.
*Files: `fluid/excitable.c`, `fluid/reaction_wave.c`*

---

#### E9 Marching Squares — Scalar Field Contour Extraction

Marching Squares extracts iso-contours from a 2D scalar field by classifying each 2×2 cell of grid corners by a 4-bit index: each bit is 1 if that corner's value exceeds the threshold, 0 otherwise. The 16 possible patterns are handled by a precomputed lookup table that specifies which edges the contour must cross. Edge crossings are linearly interpolated:

```
t = (threshold − f_a) / (f_b − f_a)
crossing = a + t · (b − a)
```

Two ambiguous cases exist (patterns 5 and 10 — diagonally opposite corners both inside) where the topology is underdetermined. The standard disambiguation tests the field value at the cell centre to decide which of two possible contour topologies applies.

For ASCII rendering, the crossing direction (the slope of the line segment through the cell) maps to a character: nearly horizontal → `─`, nearly vertical → `│`, diagonals → `/` `\`, corners → `+`. The metaball potential field `f(x,y) = Σ Aᵢ / rᵢ²` (gravitational potential of multiple point masses) is the canonical test signal — two nearby blobs produce an organic "peanut" contour that merges into a single blob as they approach.
*Files: `fluid/marching_squares.c`*

---

### L — Algorithms & Data Structures (new entries)

#### L9 Graham Scan & Jarvis March — Convex Hull

The convex hull of N points is the smallest convex polygon containing all points. Two classic algorithms offer different complexity profiles:

**Graham scan (O(N log N)):** Choose the lowest-leftmost point as pivot. Sort remaining points by polar angle from the pivot. Sweep through the sorted list maintaining a stack — pop the top whenever the last three points form a clockwise (non-left) turn (cross product ≤ 0), then push the new point. The stack contains the hull in CCW order.

**Jarvis march / gift wrapping (O(N·h)):** Start at the leftmost point. At each step, find the most counter-clockwise point from the current hull vertex by comparing all pairs with the cross product. Add it and advance. Stop when the hull closes. Optimal when `h ≪ N`; degrades to O(N²) when all points are on the hull.

The cross product `(A→B) × (A→C) = (Bx−Ax)(Cy−Ay) − (By−Ay)(Cx−Ax)` is the foundation of all computational geometry predicates: positive → left turn (CCW), zero → collinear, negative → right turn (CW). Floating-point near-collinear points may give wrong signs — robust implementations use exact arithmetic or add a small epsilon to the comparison.
*Files: `geometry/convex_hull.c`*

---

#### L10 BFS / DFS / A* Graph Search

Three graph traversal algorithms with distinct data structures drive their frontier expansions: BFS uses a FIFO queue (first discovered = first expanded), producing level-by-level wavefronts that guarantee shortest paths in unweighted graphs. DFS uses a LIFO stack (most recently discovered = first expanded), following one path as deep as possible before backtracking — efficient for reachability but not for shortest paths. A* uses a min-heap keyed by `f(n) = g(n) + h(n)` where `g` is the known cost from start and `h` is an admissible heuristic — it expands the node with the lowest estimated total cost first, finding optimal paths with far fewer expansions than BFS in practice.

For grid graphs, the Manhattan distance heuristic `h = |nx − gx| + |ny − gy|` is admissible (never overestimates) for 4-directional movement. Euclidean distance is admissible for 8-directional or graph-node layouts. When `h = 0`, A* degenerates to Dijkstra's algorithm. When `h` overestimates, the path may be suboptimal but finds it faster.

Force-directed layout (Fruchterman-Reingold) positions graph nodes by simulating repulsive forces between all pairs and attractive spring forces along edges: `F_repel ∝ k²/d`, `F_attract ∝ d²/k`, where `k = √(area/N)`. Iterating to equilibrium produces visually legible layouts where connected nodes cluster and unconnected nodes separate.
*Files: `artistic/graph_search.c`*

---

#### L11 DFS Maze Generation — Recursive Backtracker

A perfect maze (exactly one path between any two cells) is a spanning tree of the grid graph. The recursive backtracker builds it with DFS: start at a random cell, mark it visited, randomly choose an unvisited neighbour, remove the wall between them, and recurse. When all neighbours are visited, backtrack. The process terminates when every cell has been visited.

Walls are stored as a 4-bit bitmask per cell (N=1, E=2, S=4, W=8). Carving a wall clears the corresponding bit in the current cell and sets the opposite bit in the neighbour — ensuring symmetric representation. Animated generation processes `GEN_STEPS` DFS steps per frame so the maze grows visibly. BFS solving is then run on the completed maze, using the carved-wall bitmask as the adjacency test: a move from `(r,c)` to `(r-1,c)` is valid if `walls[r][c] & N`.

DFS mazes have long winding corridors with few dead ends — they are visually "river-like." Wilson's algorithm produces uniformly random spanning trees (all spanning trees equally likely), producing mazes with shorter average path lengths and more dead ends.
*Files: `misc/maze.c`*

---

#### L12 Sorting Algorithm Step-Iterator — Coroutine Pattern

Sorting algorithms are naturally sequential: the algorithm runs to completion in one pass. Visualization requires pausing after each comparison or swap. The standard approach avoids threads or setjmp coroutines by implementing each algorithm as a state machine struct with a `step()` function that advances exactly one operation:

```c
typedef struct { int phase, i, j; /* algorithm state */ } SortState;
bool bubble_step(SortState *s, int *arr, int n); /* returns false when done */
```

The outer animation loop calls `step()` at a controlled rate, updating `highlight[cmp1, cmp2]` to show the currently active pair in a contrasting color. This gives the user control over animation speed (steps per frame) without restructuring the algorithm itself. Each of the five algorithms (bubble, insertion, selection, quicksort with explicit stack, heapsort) is a separate state machine — they can run concurrently in split-screen mode since each has independent state.
*Files: `misc/sort_vis.c`*

---

#### L13 Hexagonal Grid Coordinate Systems

Hexagonal grids have three common coordinate systems. **Offset coordinates** (used in `hex_life.c` and `hex_grid.c`) store cells in a rectangular 2D array with alternate rows shifted — easy to store but awkward for neighbour arithmetic that depends on row parity. **Axial / cube coordinates** use three axes `(q, r, s)` with the constraint `q + r + s = 0`, allowing all six neighbours to be expressed as constant offsets regardless of position: the six directions are `(±1,∓1,0)`, `(±1,0,∓1)`, `(0,±1,∓1)`. Ring distance is `max(|q|,|r|,|s|)`.

Converting between axial and screen coordinates: `screen_col = q + (r - (r & 1)) / 2`, `screen_row = r`. The terminal aspect correction applies: a hexagonal "circle" of radius `R` in axial space appears on screen as an ellipse of width `2R` columns and `R` rows, because cells are twice as tall as wide. Scaling axial `q` by `CELL_H / CELL_W ≈ 2` before screen conversion produces visually round hex grids.
*Files: `geometry/hex_grid.c`, `artistic/hex_life.c`*

---

### N — Flocking & Collective Behaviour (new entries)

#### N7 Shepherd / Herding — Flee Force with Panic Boost

The herding simulation adds a user-controlled shepherd to the Classic Boids model. Sheep experience five combined forces: the three standard Boids forces (separation, alignment, cohesion) plus a flee force from the shepherd and a boundary containment force. The flee force uses inverse-distance weighting: `F_flee = (pos_sheep − pos_shepherd) / dist` so nearby shepherds produce overwhelming repulsion while distant ones are barely felt.

A panic zone at `PANIC_RADIUS < FLEE_RADIUS` triples the flee weight — sheep sprint when the shepherd is close. This creates two behavioural modes: gentle steering (shepherd at medium range) and panicked scattering (shepherd closes in). The combination of cohesion (which tries to keep sheep together) and flee (which pushes them apart) produces the realistic oscillation where a flock partly stays together and partly scatters when a sheepdog approaches.

Sheep use elastic wall bounces (velocity component reflected) rather than toroidal wrap. This makes them cornerable — the flock can be funnelled through an opening, which is the fundamental herding technique. Toroidal wrap would let sheep escape through walls, making herding impossible.
*Files: `flocking/shepherd.c`*

---

#### N8 Ant Colony Optimization — Stigmergic Pheromone Trails

Ant Colony Optimization (ACO) solves combinatorial optimisation problems by simulating how ants collectively find shortest paths through indirect environmental communication (stigmergy). Each ant deposits pheromone on its path; future ants prefer paths with stronger pheromone. Shorter paths are traversed more frequently, accumulate pheromone faster, and attract exponentially more ants — a positive feedback loop that converges on near-optimal routes.

The pheromone update rule is: `τ(t+1) = (1-ρ)·τ(t) + Σ Δτᵏ` where `ρ` is the evaporation rate and `Δτᵏ = Q/L_k` for each ant that used the edge. Evaporation is essential — without it, pheromone only accumulates and the system cannot forget suboptimal paths. Movement probability is `P(i→j) ∝ τᵢⱼᵅ · ηᵢⱼᵝ` where `η = 1/distance` is the heuristic and `α, β` control the exploitation/exploration balance.

The terminal implementation uses a 2D grid rather than a complete graph, making ant movement visual: trails form as visible density gradients in the pheromone field. The pheromone concentration at each cell maps directly to the Bourke ASCII density ramp — denser trails appear as denser characters, making the path selection dynamics directly observable.
*Files: `artistic/ant_colony.c`*

---

### P — Fractal Systems (new entries)

#### P16 Apollonian Gasket — Descartes Circle Theorem

An Apollonian gasket is a fractal generated by starting with three mutually tangent circles and recursively filling every gap with the unique circle tangent to all three boundary circles. The key is Descartes' Circle Theorem: if four mutually tangent circles have curvatures `k₁, k₂, k₃, k₄` (curvature = 1/radius, negative for the outer enclosing circle), then:

```
(k₁ + k₂ + k₃ + k₄)² = 2(k₁² + k₂² + k₃² + k₄²)
```

Solving for `k₄`: `k₄ = k₁ + k₂ + k₃ ± 2√(k₁k₂ + k₂k₃ + k₃k₁)`. The complex Descartes theorem gives the new circle centre directly: `k₄z₄ = k₁z₁ + k₂z₂ + k₃z₃ ± 2√(k₁k₂z₁z₂ + k₂k₃z₂z₃ + k₃k₁z₃z₁)`. Two solutions exist — one is the already-known circle, the other is the new circle.

The integer Apollonian gasket starting configuration `(k = −1, 2, 2, 3)` keeps all curvatures integer throughout the recursion — a number-theoretic curiosity. Circle count grows as `3^depth`; the recursion terminates when the resulting radius falls below one pixel. Terminal rendering draws each circle as a ring of characters at positions satisfying `||P − centre|| − radius| < 0.7 px`, with terminal aspect correction applied to the radius test in the row direction.
*Files: `fractal_random/apollonian.c`*

---

#### P17 Logistic Map Bifurcation Diagram — Feigenbaum Scaling

The logistic map `xₙ₊₁ = r·xₙ·(1−xₙ)` models population dynamics. As `r` increases from 2.5 to 4, the long-term attractor undergoes period-doubling bifurcations: a single fixed point at `r < 3`, period-2 orbit at `r ≈ 3`, period-4 at `r ≈ 3.449`, period-8 at `r ≈ 3.544`, accumulating at `r∞ ≈ 3.5699456...` (onset of chaos). The Feigenbaum constant `δ ≈ 4.669` is the ratio of successive bifurcation intervals — universal across all unimodal maps, not just the logistic map.

The diagram is computed column by column: for each column mapped to a value of `r`, run `WARMUP = 500` transient iterations to reach the attractor, then plot the next `PLOT = 300` values as dots. No array storage is needed — `mvaddch` plots each value directly. The diagram reveals self-similar structure: the same bifurcation tree reappears inside the chaotic regions, scaled by `δ` at each level.

Auto-zoom scrolls the viewport toward `r∞`, progressively revealing the self-similar structure at finer and finer scales. Panning and zooming are implemented by adjusting the `r_min, r_max` bounds — each zoom doubles the resolution of the displayed bifurcation tree.
*Files: `fractal_random/bifurcation.c`*

---

#### P18 Burning Ship Fractal — Absolute-Value Fold

The Burning Ship fractal modifies the Mandelbrot iteration `z ← z² + c` with one change: before squaring, take the absolute value of both real and imaginary components: `z ← (|Re(z)| + i|Im(z)|)² + c`. Expanded: `Re_new = Re² − Im² + Re(c)`, `Im_new = 2|Re|·|Im| + Im(c)`. The `|Im|` term forces the imaginary component positive after every step, breaking the left-right symmetry and creating asymmetric "flame" shapes rather than Mandelbrot's symmetric bulbs.

Negating the imaginary axis before display (`cy = −(py − centre_y) × scale`) flips the image so the characteristic ship silhouette appears hull-down (the way the name derives from the fractal's appearance). Smooth colouring applies the same fractional escape formula as the Mandelbrot set: `mu = iter + 1 − log₂(log₂|z|)`, producing smooth gradients across escape-time iso-shells.

Geometrically the `|·|` operation reflects the complex plane into the first quadrant before each squaring. This is topologically a fold — the Julia sets become asymmetric and the main cardioid transforms into the pointed ship hull with mast and flames. Self-similar miniature copies of the ship appear along the boundary at ever smaller scales, as in the Mandelbrot set.
*Files: `fractal_random/burning_ship.c`*

---

#### P19 Dragon Curve — Paper-Folding Sequence

The dragon curve is the fractal traced when you fold a strip of paper in half repeatedly in the same direction and then unfold it so every crease is exactly 90°. After `n` folds the strip has `2ⁿ` segments and `2ⁿ − 1` turns. The turn sequence is computed without storing the full string using the paper-folding bit trick: turn `i` (1-indexed) is RIGHT if `((i & -i) << 1) & i` is non-zero, LEFT otherwise. This extracts whether the bit above the lowest-set-bit of `i` is 1.

The sequence obeys a self-similar recursive structure: the turn sequence for generation `n+1` is `T_n R complement-reverse(T_n)` — insert a RIGHT turn in the middle and append a reversed, complemented copy of the previous sequence. This construction proves that the dragon curve never self-intersects: each turn is uniquely determined and no segment is ever revisited.

Four copies of the dragon curve rotated by 0°, 90°, 180°, 270° tile the plane without gaps or overlaps — it is a rep-tile of order 4. Terminal rendering uses Bresenham line drawing for each segment, choosing `/`, `\`, `|`, `−` by the segment slope direction, with aspect correction applied to horizontal steps (`×CELL_H/CELL_W`) to prevent circles becoming ellipses.
*Files: `fractal_random/dragon_curve.c`*

---

#### P20 Newton Fractal — Basin of Attraction Coloring

Newton's method for root-finding applied per-pixel in the complex plane creates a fractal boundary between basins of attraction. For a polynomial `f(z)`, the iteration `z ← z − f(z)/f′(z)` converges to one of the polynomial's roots for most starting points. The basin of attraction of root `rᵢ` is the set of starting points that converge to `rᵢ`. The boundaries between basins are infinitely intricate — no matter how fine the zoom, the boundary remains fractal.

For `f(z) = z⁴ − 1` (four roots at `±1, ±i`), each root's basin is colored with a distinct hue; brightness encodes convergence speed (fewer iterations → lighter). Points near basin boundaries converge slowly and appear dark — these are the fractal edges where the iteration visits many roots before settling. Near a critical point (where `f′(z) ≈ 0`), the iteration takes a huge step and may converge to a completely different root than a nearby pixel.

Damped Newton (`z -= α·f(z)/f′(z)` with `α < 1`) slows convergence and exposes finer boundary structure. The four roots of unity form a 4-fold symmetric fractal; the basins have fractal dimension between 1 and 2 — they are neither curves nor areas but something in between.
*Files: `fractal_random/newton_fractal.c`*

---

#### P21 Strange Attractors — Density-Map Rendering

Strange attractors (Clifford, de Jong, Lorenz projected to 2D) are visualised by iterating the attractor map for millions of steps and accumulating visit counts in a density grid rather than plotting each point directly. The Clifford map: `x′ = sin(a·y) + c·cos(a·x)`, `y′ = sin(b·x) + d·cos(b·y)`. Different parameter sets `(a, b, c, d)` produce completely different shapes — from simple loops to intricate filamentary structures.

Log normalisation is essential: `brightness = log(1 + count) / log(1 + max_count)`. The spine of the attractor accumulates counts orders of magnitude higher than outlying filaments. Linear normalisation makes the spine pure white and filaments invisible; log normalisation compresses the dynamic range so both are visible simultaneously.

Warm-up iterations (first 5000 discarded) move the orbit onto the attractor before plotting begins. The invariant measure of a strange attractor has multi-fractal structure — different regions have different fractal dimensions. This is visible in the density map as regions of varying texture density. Interactive parameter sliders let the user explore the parameter space and watch the attractor shape morph continuously between configurations.
*Files: `fractal_random/strange_attractor.c`*

---

#### P22 Perlin Noise Landscape — Parallax Terrain Scrolling

Fractional Brownian Motion (fBm) terrain is constructed by summing `OCTAVES` independent Perlin noise samples at doubling frequencies and halving amplitudes: `h(x) = Σ (0.5ᵏ · noise(2ᵏ · x))`. Low-frequency octaves provide the large-scale ridge structure; high-frequency octaves add fine detail. The result has a 1/f power spectrum — the same statistical self-similarity found in real terrain.

Parallax scrolling simulates depth perception by scrolling each layer at a speed proportional to its perceived distance. Background mountains (slowest, lowest frequency, brightest) scroll at 12% of camera speed; midground hills at 38%; foreground terrain (fastest, highest frequency, darkest) at 100%. Drawing layers back-to-front ensures near terrain occludes far terrain naturally. The color scheme encodes altitude: water (blue) → plains (green) → hills (yellow) → peaks (white).

The camera scrolls by advancing the noise sample coordinate: `x_sample = col × FREQ_SCALE + g_scroll`. Because Perlin noise is defined everywhere on the real line, there is no tiling seam — the landscape can scroll indefinitely without repetition (until floating-point precision is exhausted at large `g_scroll` values, typically after many minutes).
*Files: `fractal_random/perlin_landscape.c`*

---

#### P23 Involute Gear Tooth Geometry

An involute gear tooth profile is derived from the involute of the base circle: the curve traced by the end of a taut string unwound from the circle. Parametrically: `x = r·(cos(t) + t·sin(t))`, `y = r·(sin(t) − t·cos(t))`. This geometry ensures that meshing gears maintain a constant velocity ratio — the contact point moves along a fixed pressure line regardless of where in the mesh cycle the teeth are. For ASCII rendering, the gear is not drawn with the full involute profile; instead the tooth boundaries are approximated as radial line segments and circular arcs, which are close enough to the involute at terminal resolution.

Sparks emitted from tooth tips carry the tangential surface velocity `v_tang = −sin(θ) × ω × R` where `θ` is the tooth tip angle and `ω` is the angular velocity. At low `ω` sparks fly nearly radially; at high `ω` the tangential component dominates and sparks sweep in wide arcs that track the rotation direction. This is a physically accurate model of grinding sparks — the same phenomenon seen on angle grinders and lathes.

Wireframe rendering without rasterization: each terminal cell is tested for proximity to circular arc or radial edge features using polar coordinates `(rad, ang)`. Circle arcs use `|rad − R_target| < THRESH_CIRC`; radial edges use the angle from the nearest tooth centre. This avoids any polygon rasterization — the gear is defined by distance-to-feature inequalities evaluated per-cell.
*Files: `artistic/gear.c`*

---

### Q — Artistic / Signal-Based Effects (new entries)

#### Q5 Cooley-Tukey Radix-2 FFT Visualizer

The Cooley-Tukey FFT recursively splits an N-point DFT into two N/2-point DFTs of even- and odd-indexed samples, then combines them with twiddle factors `W_N^k = exp(−2πik/N)`:

```
X[k]     = E[k] + W_N^k · O[k]     k = 0 … N/2−1
X[k+N/2] = E[k] − W_N^k · O[k]
```

The iterative bottom-up implementation avoids stack overhead. Input samples are reordered by bit-reversal permutation first (so index `n` appears at position `bit_reverse(n)`), then the butterfly stages process groups of increasing span `m = 2, 4, 8, …, N`. Complexity: O(N log₂ N) vs O(N²) for the naive DFT — for N=256 that is 2048 vs 65536 multiplications.

Only the first N/2 bins are displayed (the upper half are complex conjugates of the lower half for real-valued inputs — the Nyquist theorem). Bin `k` represents frequency `k·f_sample/N`. Adding a pure sine of frequency `f` to the signal produces a spike at bin `f` — the DFT is linear, so a sum of three sines produces three spikes. Spectral leakage appears when a sine frequency falls between two bins: energy "leaks" into adjacent bins and the spike becomes a wide lobe. Multiplying the input by a window function (Hann, Hamming) reduces leakage at the cost of frequency resolution.
*Files: `artistic/fft_vis.c`*

---

#### Q6 DFT Epicycles — Fourier Drawing Replay

Any closed 2D curve sampled at N points can be decomposed into N complex DFT coefficients, each representing a rotating circle (epicycle) with frequency `n`, amplitude `|Z[n]|/N`, and initial phase `arg(Z[n])`. Treating each point `(x_k, y_k)` as a complex number `z_k = x_k + iy_k`, the DFT is:

```
Z[n] = Σ_{k=0}^{N-1} z[k] · exp(−2πikn/N)
```

Reconstruction at time `t ∈ [0,1]` is `z(t) = Σ_n (Z[n]/N)·exp(2πint)` — the sum of N rotating arms. Sorting arms by decreasing amplitude `|Z[n]|` gives the energy-optimal partial approximation: the first `M` arms reconstruct the dominant shape, successive arms add finer detail. Parseval's theorem ensures the cumulative power fraction `Σ_{k=0}^{M} |Z[k]|² / Σ |Z[k]|²` rises monotonically from 0 to 1 as `M` increases.

The Gibbs phenomenon is visible on shapes with sharp corners (squares, arrows): the partial Fourier sum overshoots the true value by approximately 9% at each discontinuity, regardless of how many terms are included. This overshoot does not decrease with more terms — it merely concentrates into a narrower spike. Aspect correction is applied to the `x` coordinates before the DFT so the reconstructed shape appears undistorted on the non-square terminal grid.
*Files: `artistic/fourier_draw.c`*

---

### T — Wave Physics & Signal Analysis (new entries)

#### T4 2D Wave Equation — FDTD with Sponge Boundaries

The 2D wave equation `∂²u/∂t² = c²∇²u` is discretized with the second-order FDTD stencil:

```
u[t+1][i][j] = 2u[t][i][j] − u[t−1][i][j]
             + C²·(u[t][i+1][j] + u[t][i−1][j]
                  + u[t][i][j+1] + u[t][i][j-1] − 4u[t][i][j])
             − DAMP·(u[t][i][j] − u[t−1][i][j])
```

The CFL stability condition for 2D is `c·dt/dx ≤ 1/√2`. Violating it causes exponential blow-up within a few steps. Only two time levels (`t` and `t−1`) are needed despite the second-order time derivative — the stencil computes `t+1` from both, then the old `t−1` buffer is overwritten with `t+1`.

A sponge (absorbing) boundary layer at the grid edges suppresses reflections that would otherwise create a standing-wave "box mode." Within `BORDER_W` cells of each edge, the damping coefficient ramps up from zero at the interior boundary to a maximum at the wall. This is far simpler to implement than a Perfectly Matched Layer (PML) and sufficient for terminal-resolution wave visualisation. Point sources are driven as `u[row][col] += A·sin(2π·f·t)`, creating circular wave-fronts; multiple sources create interference patterns where constructive and destructive interference alternate spatially.
*Files: `fluid/wave_2d.c`*

---

#### T5 Navier-Stokes Stable Fluids — Stam's Method

Jos Stam's "Stable Fluids" (SIGGRAPH 1999) simulates incompressible viscous flow by operator-splitting each timestep into four sub-steps, each of which is unconditionally stable regardless of timestep size:

1. **Add forces:** `v += f · dt`
2. **Diffuse:** `(I − ν∇²)v_new = v` — solved with Gauss-Seidel iteration on the implicit system. Unconditionally stable because the implicit system always has a solution.
3. **Advect:** `v(x, t+dt) = v(x − v·dt, t)` — semi-Lagrangian back-tracing. Sample the field at the back-traced position with bilinear interpolation. Unconditionally stable because it is a sampling operation, not an extrapolation.
4. **Project:** Solve the Poisson equation `∇²p = ∇·v` with Gauss-Seidel, then subtract `∇p` from velocity. This enforces incompressibility (`∇·v = 0`).

The density field (dye/smoke) follows the same diffuse+advect steps without the project step. Dynamic normalisation (`display = density / max_density`) prevents the display from going blank as dye concentrations change. Gauss-Seidel with `ITER = 16` iterations gives acceptable incompressibility for N = 80: residual decays geometrically per pass and the visual error is imperceptible.
*Files: `fluid/navier_stokes.c`*

---

#### T6 FitzHugh-Nagumo Reaction-Diffusion — Excitable Media

The FitzHugh-Nagumo equations model excitable media such as cardiac muscle and nerve axons. Two coupled PDEs govern a fast activator `u` (membrane voltage analogue) and a slow inhibitor `v` (recovery variable):

```
∂u/∂t = u − u³/3 − v + D∇²u   ← fast, cubic nonlinearity creates threshold
∂v/∂t = ε·(u + a − b·v)       ← slow recovery
```

The cubic term `u³/3` creates the excitation threshold: small perturbations decay back to rest; perturbations above threshold trigger a full spike. The inhibitor `v` rises after each spike (refractory period) and prevents re-excitation — this is why action potentials travel as one-way waves rather than reflecting back. The `ε ≪ 1` timescale ratio makes `v` much slower than `u`, producing the characteristic fast-rise, slow-decay shape of an action potential.

Explicit Euler integration with `STEPS_PER_FRAME = 8` sub-steps maintains the CFL stability condition `DT·D/dx² < 0.25` while displaying at 30 fps. Spiral waves initiate from a broken planar wavefront: create a horizontal wave then make the lower half of the medium refractory. The free end curls into a rotating spiral that sustains itself indefinitely. Two counter-rotating spirals that collide annihilate each other — a direct simulation of cardiac re-entry arrhythmia termination.
*Files: `fluid/reaction_wave.c`, `fluid/excitable.c`*

---

### I — Raytracing (new entries)

#### I14 Analytic Ray-Capsule Intersection

A capsule is a cylinder capped at each end by a hemisphere. The analytic intersection decomposes into two sub-problems solved sequentially. First, project out the axial component of the ray to reduce to a 2D problem:

```
ba = cb − ca      (axis vector)
a  = baba − bard²  (baba = |ba|², bard = dot(ba,rd))
b  = baba·dot(rd,oa) − baoa·bard
c  = baba·(|oa|² − r²) − baoa²
h  = b² − a·c
```

If `h < 0`, the ray misses the infinite cylinder and both caps. Otherwise, candidate body hit at `t = (−b − √h)/a`. The body hit is valid if the axial coordinate `y = baoa + t·bard` satisfies `0 < y < baba` (between the two cap planes). If the body misses, try the hemisphere at the corresponding endpoint using a standard sphere quadratic. The body-to-cap normal transition is `C¹` continuous: at the seam the radial body normal and the hemispherical cap normal are equal.

The key optimisation is that if `h < 0` (cylinder miss), no cap can be hit — caps are always geometrically inside the cylinder's lateral extent. This early-exit saves two square roots on the common miss path. The same inverse-rotation-matrix trick used in `cube_raytrace.c` transforms the ray into object space so the capsule axis is always aligned with Y — no general axis-handling is needed.
*Files: `raytracing/capsule_raytrace.c`*

---

#### I15 Quartic Torus Ray Intersection — Sampling + Bisection

Substituting the ray `P(t) = ro + t·rd` into the torus implicit equation `(√(x²+z²) − R)² + y² = r²` and expanding produces a quartic polynomial in `t`:

```
t⁴ + A·t³ + B·t² + C·t + D = 0
A = 4·dot(rd,ro)
B = 4·dot(rd,ro)² + 2·C₀ − 4R²·(rdx² + rdz²)
C = 4·dot(rd,ro)·C₀ − 8R²·(rdx·rox + rdz·roz)
D = C₀² − 4R²·(rox² + roz²)    where C₀ = |ro|² + R² − r²
```

Rather than Ferrari's analytic quartic solver (numerically unstable near degenerate cases), a hybrid sampling+bisection approach is used: evaluate the quartic via Horner's method `t(t(t(t+A)+B)+C)+D` at 256 equally-spaced points in `[ε, T_MAX]`; detect sign changes; bisect 40 times in each bracketed interval to locate the root to sub-floating-point precision. This is robust, predictable, and fast enough for terminal frame rates.

The surface normal at hit point `P` is the gradient of the implicit function: `N = normalize(P − R·normalize(P_xz))` where `P_xz = (P.x, 0, P.z)` is the XZ projection. Geometrically, `R·normalize(P_xz)` is the nearest point on the ring centreline — the normal points radially outward from that centre-line point, which is the "outward tube direction" at any surface location.
*Files: `raytracing/torus_raytrace.c`*

---

---

#### D17 Euler-Bernoulli Beam — Analytical Statics + Modal Dynamics

The Euler-Bernoulli beam model assumes cross-sections remain plane and perpendicular to the neutral axis. The governing equation for transverse deflection `w(x)`:

```
EI · d⁴w/dx⁴ = q(x)
```

where `E` is Young's modulus, `I` is the second moment of area, and `q(x)` is the distributed load. Integrating four times with appropriate boundary conditions gives closed-form `w(x)`. The bending moment `M(x) = EI · w''` (second derivative) follows from the deflection profile.

**Nine analytical solutions** are tabulated for 3 BC types × 3 load types:

- Simply supported (SS): zero deflection and zero moment at both ends
- Cantilever: fixed at x=0 (zero deflection + zero slope), free at x=L
- Fixed-fixed: zero deflection and zero slope at both ends

For each BC, three load cases are solved: concentrated center load P, uniformly distributed load q, and end moment M.

**Modal dynamics** use the free-vibration eigenmodes of each BC type. For a cantilever, the characteristic equation is `1 + cos(βL)·cosh(βL) = 0`, giving β₁L = 1.8751, β₂L = 4.6941, etc. The eigenfrequency is `ω_n = (β_nL)²·√(EI/ρA·L⁴)`. Each mode runs as an independent damped oscillator solved with the exact transition matrix (avoids explicit-Euler instability at high ω·dt).

**Why exact transition matrix instead of Euler integration:**
The modal ODE `q̈ + 2ζω_n·q̇ + ω_n²·q = F_n(t)` has a closed-form solution per step:

```
q(t+dt) = q_s + e^(−ζωdt) · [A·cos(ωd·dt) + B·sin(ωd·dt)]
```

where `q_s = F_n/ω_n²` is the static equilibrium, `ωd = ω_n√(1−ζ²)`, and A, B are set from initial conditions. This is unconditionally stable for any dt — a property explicit Euler lacks (it diverges when `ω_n·dt > 2`).

*Files: `physics/beam_bending.c`*

---

#### D18 Differential Drive Robot — Nonholonomic Kinematics

A differential drive robot has two wheels on a shared axle, driven independently. The robot's configuration is the pose (x, y, θ). From wheel velocities:

```
v  = (vL + vR) / 2          center velocity
ω  = (vR − vL) / L          angular velocity (axle width L)
ẋ  = v · cos(θ)
ẏ  = v · sin(θ)
θ̇  = ω
```

**Nonholonomic constraint:** the robot cannot move perpendicular to its heading. This is encoded implicitly — velocity is always (v·cosθ, v·sinθ), no lateral component. The constraint is automatically satisfied without any penalty or projection step.

**Instantaneous Centre of Curvature (ICC):** when ω ≠ 0, the robot turns about a point at distance `R = v/ω` from the robot's midpoint. When vL = vR the robot goes straight (R → ∞). When vL = −vR the robot spins in place (R = 0).

**Perpendicular wheel geometry in y-down coordinates** (standard terminal layout, +y points down):

```
heading direction:  (cos θ, sin θ)
perpendicular left: (+sin θ, −cos θ)     [north of body when facing east]
perpendicular right:(−sin θ, +cos θ)     [south of body when facing east]
```

A common mistake is to use the screen-geometry perpendicular without accounting for the y-down flip. The correct form has the +sin/−cos sign pattern, not the standard math-convention −sin/+cos.

**Why V_DECAY = 1.0 (no linear decay):** A real wheeled robot on flat ground has negligible rolling resistance at low speeds. Setting decay < 1.0 per tick at 60 Hz creates exponential drag that makes the robot stop within a second of key release — counterintuitive and physically wrong for a flat surface. The user controls speed explicitly; braking requires the dedicated S key.

*Files: `robots/diff_drive_robot.c`*

---

## D19 Inverted Pendulum — Lagrangian Cart-Pole on Slope

The inverted pendulum is the canonical benchmark for control theory. A pole of length L is hinged at the top of a cart. The goal is to keep it upright by accelerating the cart horizontally.

**Lagrangian derivation on flat ground:** With generalised coordinates cart position x and pole angle θ (measured from vertical), the Lagrangian L = T − V yields the coupled EOM. After eliminating the cart equation the pole dynamics reduce to:

```
θ_ddot = (g·sin(θ) − ẍ·cos(θ)) / L
```

This is the core equation: gravity pulls the pole over (`g·sin(θ)`), and cart acceleration pushes it back (`−ẍ·cos(θ)`).

**Slope generalisation:** On terrain with slope angle α the effective angle is `θ_eff = θ − α`. The equilibrium is no longer vertical — the bot must lean into the slope. The equation becomes:

```
θ_ddot = (g·sin(θ_eff) − ẍ·cos(θ_eff)) / L
```

The slope thus enters purely as an offset to θ in both the sin and cos terms.

**Why linearise around zero?** For small angles sin(θ) ≈ θ and cos(θ) ≈ 1, giving `θ_ddot ≈ (g·θ − ẍ)/L`. This is the linear model standard PID is tuned against. The full nonlinear form is used here for visual correctness at larger angles.

**Numerical integration:** Symplectic Euler (update ω first, then θ from updated ω) is unconditionally energy-preserving for the undamped case and is preferred over explicit Euler which would inject energy and cause instability.

*Files: `robots/perlin_terrain_bot.c`*

---

## D20 PID Controller — Proportional/Integral/Derivative with Windup Clamp

A PID controller closes a feedback loop by computing a corrective action from three terms acting on the error signal `e = setpoint − measurement`.

**The three terms:**

```
u(t) = Kp·e(t)  +  Ki·∫e dt  +  Kd·(de/dt)
```

- **P (proportional):** Immediate reaction proportional to current error. High Kp gives fast response but causes overshoot and oscillation.
- **I (integral):** Accumulates past error to eliminate steady-state offset. Slow to act; too high causes "integral windup" where the integrator saturates and the controller overshoots badly on recovery.
- **D (derivative):** Reacts to the rate of change of error, damping oscillations. Sensitive to noise; too high amplifies measurement noise.

**Integral windup clamp:** The integral accumulator is clamped to `[-IMAX, +IMAX]` every tick. Without this, a sustained large error (e.g. the bot hitting a steep slope and falling) saturates the integrator. When the error finally reverses, the clamped integrator lets the controller recover quickly instead of fighting a large stored charge.

**Cascade control (position + angle):** The outer loop is position control: an error in horizontal speed or position generates a desired lean angle. The inner loop is angle control: the angle error drives the wheel torque (cart acceleration `ẍ`). The slope feed-forward `θ_ref = −α×0.65` is added to the outer-loop setpoint so the bot pre-leans into the hill rather than waiting for the angle error to build up.

**Gain preset teaching:** The six named presets in `perlin_terrain_bot.c` show characteristic PID failure modes in the phase portrait:
- HIGH Kp: fast but oscillatory spiral in phase space
- LOW Kp: sluggish, large excursion before settling
- NO Kd: underdamped, persistent oscillation
- HIGH Kd: overdamped, slow creep to equilibrium
- NO Ki: stable but with persistent lean offset on slope

*Files: `robots/perlin_terrain_bot.c`*

---

---

## D21 Vorticity-Streamfunction (VSF) Navier-Stokes

The vorticity-streamfunction (ω-ψ) formulation eliminates pressure from 2-D incompressible Navier-Stokes, leaving two scalar PDEs instead of the coupled velocity-pressure system.

**Why eliminate pressure?** In the primitive variable form (u, v, p) you must solve a Poisson equation for pressure at every step to enforce ∇·u=0 (incompressibility). Taking the curl of momentum eliminates ∇p entirely:

```
∂ω/∂t + u·∇ω = ν·∇²ω    (vorticity transport)
∇²ψ = −ω                   (Poisson: streamfunction)
u = ∂ψ/∂y,   v = −∂ψ/∂x  (velocity recovery)
```

You still solve a Poisson equation (for ψ), but only once per step rather than iteratively within a pressure-correction loop.

**SOR vs Gauss-Seidel:** Plain Gauss-Seidel has spectral radius ρ_GS ≈ 1 − π²/N² for an N×N grid — convergence is slow for large N. SOR with the optimal relaxation factor ω_opt = 2/(1+sin(π/(N+1))) gives ρ_SOR ≈ 1 − 2π/N — dramatically faster. The formula assumes a square grid with Dirichlet boundaries. For a 64×64 grid: ρ_GS ≈ 0.9976 (need ~1000 iterations) vs ρ_SOR ≈ 0.906 (need ~25 iterations).

**Thom's wall-vorticity formula (1933):** At a solid wall ψ is constant (ψ_wall=0). The vorticity is not prescribed — it must be derived from the no-slip condition. Thom's formula uses the interior streamfunction values via a one-sided second-order Taylor expansion of ∇²ψ=−ω evaluated at the wall cell:

```
ω_wall = −2·(ψ_interior − ψ_wall) / dy²
```

For the moving lid (top wall, velocity U): the term −2U/dy is added, injecting the sole vorticity source driving cavity flow.

**Upwind vs central for convection:** The nonlinear convection `u·∂ω/∂x` is linearly unstable with central differences at high Re (negative effective viscosity). Upwind differencing: `∂ω/∂x ≈ (ω_i − ω_{i-1})/dx` when u>0, `(ω_{i+1} − ω_i)/dx` when u<0. This introduces truncation-error diffusion proportional to `|u|dx/2`, which dominates the physical ν/Re at high Re — acceptable because the goal is qualitatively correct flow, not DNS accuracy.

**CFL and von Neumann stability limits:**
- Convection CFL: `dt ≤ dx / |u_max|`  (advection stability)
- Diffusion von Neumann: `dt ≤ dx² / (4ν)`  (parabolic stability)
Both must hold simultaneously. At low Re the diffusion limit is tighter; at high Re the convection CFL dominates.

*Files: `fluid/vorticity_streamfunction_solver.c`*

---

## D22 Acoustic FDTD — Leapfrog Scalar Wave Equation

The acoustic wave equation `∂²p/∂t² = c²∇²p` describes the propagation of pressure perturbations in a homogeneous medium. The FDTD discretisation is one of the most natural numerical methods for wave equations.

**Leapfrog stencil derivation:** Approximate both time and space derivatives with centred finite differences:

```
∂²p/∂t² ≈ (p_new − 2p + p_old) / dt²
∇²p     ≈ (p_{i+1,j} + p_{i-1,j} − 2p_{i,j})/dx²
          + (p_{i,j+1} + p_{i,j-1} − 2p_{i,j})/dy²
```

Substituting and solving for p_new:
```
p_new = 2p − p_old + rx²·(p_{i+1,j} − 2p + p_{i-1,j})
                    + ry²·(p_{i,j+1} − 2p + p_{i,j-1})
```
where rx² = (c·dt/dx)². This explicit update needs only the current and previous time levels — no matrix solve.

**Why leapfrog (not forward Euler)?** Forward Euler on the wave equation is unconditionally unstable — any perturbation grows. The centred-time (leapfrog) scheme is neutrally stable within the CFL limit: |A(ω)| = 1 for all frequencies when CFL ≤ 1. It introduces no numerical dissipation, preserving wave amplitude over long propagation distances.

**2-D CFL stability criterion:** In 2D the Courant number must satisfy:
```
c·dt·√(1/dx² + 1/dy²) ≤ 1
```
For a square grid dx=dy: `c·dt ≤ dx/√2`. At CFL=0.90 we use 90% of the stability limit; exceeding 1.0 causes exponential blow-up within one step.

**Terminal aspect correction:** Terminal characters are ~16px tall × 8px wide (2:1 aspect). Without correction, setting dx=dy in cell coordinates means the physical grid cells are twice as tall as they are wide. A wavefront that should be circular would appear elliptical (elongated vertically). The fix: `dy_phys = dx_phys / ASPECT_Y` where ASPECT_Y=2. This makes physical cells square, so wavefronts appear circular on screen.

**Sponge vs PML:** A perfectly matched layer (PML) achieves near-zero reflection by using a complex coordinate stretching that makes the medium impedance-matched to outgoing waves at all angles and frequencies. A sponge layer simply multiplies p by a damping factor ∈ [0,1] that ramps from 1 (interior) to 0 (boundary). The sponge is 5× simpler to implement but causes some low-frequency reflection. For visual simulations, sponge layers are sufficient.

*Files: `physics/acoustic_wavesolver.c`*

---

## D23 Lattice Boltzmann Method (D2Q9) — LBM Fluid

LBM evolved from the lattice gas automata of the 1980s. Instead of solving Navier-Stokes directly, it evolves particle distribution functions on a lattice. The macroscopic N-S equations emerge via Chapman-Enskog multiscale expansion — LBM is a mesoscopic model that recovers the correct macroscopic fluid behavior.

**Why LBM over finite-difference N-S?** LBM is trivially parallelisable (each cell's collision is independent), handles complex boundaries naturally (bounce-back), and avoids solving pressure Poisson equations. The tradeoff: it introduces a lattice Mach number constraint (Ma < 0.3) and is memory-intensive (9 floats per cell).

**D2Q9 velocity set:** The 9 lattice velocities are chosen so that the first four moments (mass, momentum, momentum flux, energy flux) of the Maxwell-Boltzmann distribution are exactly reproduced:
```
e0 = (0,0)                           w0 = 4/9
e1 = (1,0), e2=(-1,0), e3=(0,1), e4=(0,-1)    w = 1/9
e5 = (1,1), e6=(-1,1), e7=(-1,-1), e8=(1,-1)  w = 1/36
```
The lattice speed of sound cs² = 1/3 (in lattice units, dx=dt=1).

**Equilibrium distribution:** The second-order Maxwell-Boltzmann equilibrium truncated to Ma²:
```
feq_i = w_i · ρ · [1 + (e_i·u)/cs² + (e_i·u)²/(2cs⁴) − u²/(2cs²)]
```
The three polynomial correction terms are the Taylor expansion of `exp((e_i·u)/cs²)`. Only quadratic terms are kept; cubic and higher terms break Galilean invariance on the lattice.

**Viscosity-relaxation relation:** The viscosity ν = (τ − 0.5) · cs² = (τ − 0.5)/3. The constraint τ > 0.5 ensures ν > 0 (physical viscosity). The τ < 2 constraint keeps feq positive (stability). Typical range: τ ∈ [0.51, 1.8].

**Streaming race condition:** After collision, f*[x,y,i] must propagate to neighbour cell (x+e_ix, y+e_iy). If done in-place, a cell that was already updated might be read by a neighbour that has not yet been updated. The double-buffer solves this: collide from f into ftmp, then stream from ftmp into f. The two arrays swap pointers each step — O(1) overhead.

**Reynolds number:** Re = U·L/ν where U is the characteristic velocity (lattice units), L is the characteristic length (cells), ν is lattice viscosity. For flow past a cylinder of radius R: Re = U·2R/ν. Shedding threshold Re≈47; fully developed Kármán street Re≈100–300.

*Files: `physics/lattice_boltzman_fluid_simulator.c`*

---

## D24 ODE Integrator Zoo — Euler / RK2 / RK4 / Velocity Verlet

Numerical ODE integrators differ in accuracy order, computational cost (function evaluations per step), and geometric properties (symplecticity). Understanding these tradeoffs is essential for choosing the right method.

**Error order:** A method has local truncation error O(h^{p+1}) and global error O(h^p) where h is the step size. Doubling h multiplies global error by 2^p. RK4 at p=4: halving h reduces error by 16×. Euler at p=1: halving h only halves error.

**Euler — why it fails for oscillators:** The amplification matrix A maps (q,p) from one step to the next. For the harmonic oscillator |A|² = 1 + (ωh)² > 1 — eigenvalues lie outside the unit circle. Every step amplifies the state, so energy grows as (1+(ωh)²)^n ≈ exp(ω²h·t). This is not a roundoff or truncation issue — it is a fundamental stability property of the Euler scheme.

**RK2 (midpoint method):** The midpoint method achieves O(h²) by evaluating the derivative at the step midpoint and using that slope for the full step. The O(h²) leading error term in Euler (which comes from evaluating the slope only at the start) is cancelled because the midpoint slope already contains first-order information about the curvature.

**RK4 derivation sketch:** The four-stage weighted average `(k1 + 2k2 + 2k3 + k4)/6` matches the Taylor expansion of the exact solution through the h⁴ term. The coefficients 1/6, 2/6, 2/6, 1/6 are the Simpson's rule weights applied to the ODE integration interval — not a coincidence.

**Symplecticity (Verlet):** A symplectic integrator preserves the symplectic 2-form dq∧dp (volume of phase-space regions). For Hamiltonian systems this implies:
- Energy error is bounded (oscillates but does not drift)
- The invariant tori of integrable systems are preserved (Poincaré section shows closed curves, not spirals)
- No secular energy drift even over millions of steps

The velocity Verlet is symplectic because it alternates momentum half-steps with full position steps: the composition of the two half-maps is volume-preserving. By contrast, RK4 is not symplectic — it eventually drifts for very long integrations.

**When to use each:**
| Method        | Cost   | Error  | Stable for oscillators | Long-time |
|---------------|--------|--------|----------------------|-----------|
| Euler         | 1 eval | O(h)   | No (grows)            | Bad       |
| RK2           | 2 eval | O(h²)  | Borderline            | Mediocre  |
| RK4           | 4 eval | O(h⁴)  | Yes (bounded)         | Good      |
| Verlet        | 2 eval | O(h²)  | Yes (bounded)         | Excellent |

*Files: `physics/rk_method_comparision.c`*

---

## D25 Short-Time Fourier Transform — STFT Spectrogram

The STFT trades frequency resolution for time resolution relative to the full-signal DFT. A window of N samples is extracted, windowed (to reduce spectral leakage), and FFT'd. The resulting spectrum column is appended to a scrolling 2-D display.

**Time-frequency uncertainty:** The Gabor limit states Δt·Δf ≥ 1/(4π). You cannot simultaneously resolve an event in time and in frequency arbitrarily well. This is the signal-processing analog of Heisenberg's uncertainty principle (and shares the same mathematical root in Fourier analysis). In practice: N=512, SR=8000 → Δf=15.6 Hz, Δt=64ms. For transient detection use N=64; for pure tone detection use N=2048.

**Bit-reversal permutation (why DIT needs it):** The Cooley-Tukey DIT algorithm splits the input into even-indexed and odd-indexed subsequences recursively. After log₂N levels of splitting, the natural ordering of input samples is in bit-reversed order. For N=8: index 3 (binary 011) maps to index 6 (binary 110). The bit-reversal is a permutation, so it can be done in-place in O(N) by swapping x[i] with x[bitrev(i)] for i < bitrev(i).

**Butterfly stages:** Each stage performs N/2 butterfly operations: `(a, b) → (a + W·b, a − W·b)` where W = e^{−j2πk/N} is the twiddle factor for that position. The twiddle factor recurrence `W_next = W · e^{-j2π/N_stage}` avoids recomputing sin/cos in the inner loop (which would dominate cost at large N).

**Spectral leakage:** A finite-length DFT implicitly multiplies the signal by a rectangular window. A pure sine at frequency f not exactly on a DFT bin leaks energy into all other bins. The sidelobe level of the rectangular window is −13 dB — a signal 4.5× weaker than the main tone can be completely hidden. Window functions (Hann, Hamming, Blackman) taper the signal to zero at both ends, dramatically reducing sidelobes at the cost of widening the main lobe.

**AM/FM sidebands:**
- AM: `(1 + m·cos(2πfm·t))·cos(2πfc·t)` → carrier at fc + sidebands at fc±fm of amplitude m/2. Visible as three peaks in the spectrogram.
- FM: `cos(2πfc·t + β·sin(2πfm·t))` → Bessel function sideband amplitudes J_n(β) at fc±n·fm. Large β (wideband FM, β=75) gives many visible sidebands. Small β (narrowband FM) approximates AM.

*Files: `physics/spectrogram_visualizer.c`*

---

## D26 Deferred Rendering Pipeline — G-buffer + Blinn-Phong Lighting Pass

Deferred rendering decouples geometry processing from lighting. In a forward renderer, each object is shaded for all lights during geometry draw — cost is O(objects × lights). In deferred, geometry writes data to the G-buffer in pass 1, and lighting reads from it in pass 2 — cost is O(pixels × lights), independent of geometry count.

**G-buffer layout:** Each G-buffer texture stores per-pixel surface data at screen resolution:
```
RT0: world position    (x, y, z, 1)     — for light attenuation  
RT1: world normal      (nx, ny, nz, 0)  — for diffuse + specular
RT2: albedo            (r, g, b, mat)   — surface base color
RT3: material          (shininess, ...)  — specular exponent
```
This is structurally identical to Unreal Engine 5's GBufferA/B/C/D layout and Unity HDRP's RT0–RT3.

**Perspective projection depth:** The projection matrix row `m[3][2] = −1` sets the output w-component to `−z_view`. After perspective divide (NDC = clip/w), the depth buffer stores `−z_view/−z_view = 1` at the near plane and approaches 0 at the far plane (reversed-Z convention). The clip-space w = −z_view > 0 for objects in front of the camera (where z_view < 0).

**Normal matrix derivation:** When you transform a vertex by M, the normal must transform by `(M⁻¹)ᵀ` (inverse-transpose) to remain perpendicular to the surface. Proof: if n·t=0 (normal perpendicular to tangent), after transform: `(Mn·Pt) = nᵀMᵀPt`. For this to be zero for all tangents Pt = t, we need `nᵀMᵀP = nᵀ(M⁻¹)ᵀ·(MM⁻¹)ᵀP`. The shortcut: if M is a pure rotation, M⁻¹ = Mᵀ so (M⁻¹)ᵀ = M itself — no extra work needed.

**Blinn-Phong half-vector:** The Phong model reflects L around N to get R, then measures angle between R and V. The Blinn approximation uses the half-vector H = normalize(L + V): the specular term is `(N·H)^n`. H is independent of shading position (constant for parallel light and camera), making it faster for distant sources. The exponent relationship: Blinn specular exponent ≈ 4× Phong exponent to achieve the same lobe width.

**Bayer dithering vs Floyd-Steinberg:** Bayer ordered dithering uses a precomputed threshold matrix — spatially ordered, no error propagation, parallelisable. Floyd-Steinberg propagates quantisation error to neighbours — better perceptual quality but sequential (each pixel depends on previous). For real-time per-pixel rendering the Bayer approach is O(1) per pixel with no state, making it the standard choice in hardware shaders (it is used in MSAA resolve and HDR-to-LDR tone mapping).

*Files: `raster/deferred_rendering_pipeline.c`*

---

## D27 2-D Damped Wave Equation — Shockwave Demo

The damped scalar wave PDE governs everything from drumheads to acoustic shocks to ripples on a pond. In its simplest 2-D form:

```
∂²u/∂t² = c²∇²u − γ ∂u/∂t
```

The Laplacian `∇²u` is the spatial second derivative summed over both axes. On a uniform grid, the standard 5-point stencil is `u[x+1] + u[x−1] + u[y+1] + u[y−1] − 4u[x,y]` — accurate to O(dx²). Time integration is leapfrog (explicit second-order), needing only `u_old`, `u`, `u_new`. The damping term `−γ ∂u/∂t` is approximated by `−γ(u − u_old)/dt`, which is unconditionally stable at any γ.

**CFL stability** caps the wave-speed/timestep product: for the 5-point stencil in 2-D, `c · dt · √2 ≤ 1`. Two substeps per render frame halve the effective `dt`, doubling the maximum stable `c` for free — useful when you want a fast-moving wave at low render cost.

**Two decay mechanisms emerge naturally:**
- *Geometric (cylindrical spreading):* in 2-D, total energy in a ring grows linearly with the circumference, so amplitude must drop as `1/√r`. This is geometry, not damping — total energy is conserved.
- *Physical (damping):* the `γ` term converts wave energy to heat (or any sink). Amplitude envelope is `exp(−γt)`, independent of geometry.

**Initial condition matters.** A pixel-sharp delta excites every wavenumber up to Nyquist; the highest wavenumbers are exactly the unstable modes the discrete Laplacian cannot represent. A Gaussian tap of half-width 2 cells filters them out, giving a clean ring without ringing artifacts.

The `physics/nuke.c` simulation drives a coordinated visual from this single field: ring shockwave (the field itself), terrain heave (per-column displacement reading `u` at the surface row), debris/dust particle pools (spawned at blast and ring-sweep events), screen shake (integer cell offset at the `mvaddch` boundary), full-screen flash (alpha overlay).

*Files: `physics/nuke.c`*

---

## D28 Sphere Tracing & SDF Composition — Solar Simulation

Sphere tracing (Hart 1996) replaces analytic ray–surface intersection with iterative steps. Given a Signed Distance Function `f: ℝ³ → ℝ`, where `|f(p)|` equals the distance from `p` to the nearest surface and the sign tells you inside/outside, you can step along a ray by exactly `f(p)` and never overshoot. Terminate when `f(p) < ε` (hit) or accumulated `t > MAX_T` (miss).

**SDF composition** is the real power. Boolean operations are trivial:
- Union: `min(a, b)` — the closer of two surfaces
- Intersection: `max(a, b)`
- Subtraction: `max(a, −b)`

Smooth versions blend the boundary instead of butting:
- `smin(a, b, k) = min(a, b) − k·h²/4` with `h = max(k − |a−b|, 0)/k` — polynomial smooth minimum, gives a rounded crease over a region of width `k`.

**Domain operations** modify `p` before the SDF lookup: translate, rotate, repeat (`p mod size`), twist (`rotate(p.xy, p.z · k)`), bend, displace by noise. All of these compose with all primitives.

**Why noise-displaced SDFs make sense for natural surfaces.** A surface like a sun's photosphere has no analytic equation but is well-approximated by a sphere with high-frequency perturbation. Subtracting `fbm(p)·amp` from the sphere SDF gives exactly that — boiling lava without any explicit polygon mesh.

`raymarcher/sun.c` uses one SDF (`sphere − warped_fbm`) smooth-unioned with up to 8 capsule flares. The flares run independent state machines (BLAST → ARCH → DECAY); their geometry is a Bézier-arched magnetic loop approximated by 8 capsule segments (`bezier_tube_dist()`). The analytic distance to a quadratic Bézier curve requires solving a degree-5 polynomial — capsule subdivision is visually identical at one-tenth the code.

**Limb darkening** (Eddington 1-coefficient law, ref: en.wikipedia.org/wiki/Limb_darkening) accounts for the cooler-looking edge of a star: `T_visible = T·(0.80 + 0.20·μ)` with `μ = N·V`. A real photometric law is a polynomial in μ; `u₁ = 0.20` matches the visible-light limb of the Sun closely.

*Files: `raymarcher/sun.c`*

---

## D29 Volumetric Raymarching — Beer–Lambert Mushroom Cloud

Surface-based rendering (SDFs, polygons) cannot draw clouds, smoke, fog, or fire — these have no surface. Instead the medium is a 3-D scalar density field; rendering integrates light transport through the field along each ray.

**The Beer–Lambert law** describes light attenuation in a participating medium:
```
T(t) = exp(−∫₀ᵗ density(p(s)) · σ_t  ds)
```
where `T(t)` is remaining transmittance after travelling distance `t` and `σ_t` is the extinction coefficient. Discretised front-to-back:

```
T = 1
out_colour = 0
for each step along ray:
    dτ = density(p) · step · EXTINCTION
    a  = 1 − exp(−dτ)
    out_colour += a · emission(p) · T
    T *= exp(−dτ)
    if T < 0.07: break               /* invisible contribution */
```

Front-to-back (rather than back-to-front) lets you bail when `T` drops below the colour quantization threshold. The density field can be anything: voxel grid, procedural noise, or in this case, a sum of soft Gaussian primitives.

**Soft Gaussian primitives** replace SDFs for volume work. A blob is `exp(−d² · k)` where `d` is some distance metric. With *anisotropic* distance `d² = (dx/rx)² + (dy/ry)² + (dz/rz)²`, the same blob morphs from sphere → bullet → cap by independently scaling `rx`, `ry`, `rz` over time.

**Continuous-time morphing** beats scene switches. Every animated parameter is `smoothstep(t_beg, t_end, time)` over an *overlapping* window — fireball is still rising while the cap is starting to bulge. Quintic smootherstep `6t⁵−15t⁴+10t³` (zero 1st AND 2nd derivative at endpoints, Perlin SIGGRAPH 2002) on the most-watched parameter eliminates the visible acceleration kink at the seam.

**Domain warping** (Inigo Quilez, iquilezles.org/articles/warp/) generates the boiling cloud surface texture: `fbm(p + fbm2(p) · WARP_AMP)`. The self-offset creates swirling cauliflower turbulence that pure fBm cannot.

**Particle terminal velocity for ash** — without a `vy` clamp, ash accelerates indefinitely. Real particulate hits terminal velocity within a second from air drag; clamping `vy ≥ −1.6` cells/s and adding horizontal `exp(−1.4·dt)` decay produces a settling cloud that looks like real fallout, not Newtonian rocks.

`raymarcher/nuke_v1.c` puts these together: one Beer–Lambert volume integrator, one anisotropic Gaussian blob morphing through the full mushroom-cloud life cycle, three particle classes sharing one struct, 2× vertical supersampling with sub-cell glyph picker for terminal display.

*Files: `raymarcher/nuke_v1.c`*

---

*Read the code, run the programs, change one constant at a time. That is how it becomes yours.*
