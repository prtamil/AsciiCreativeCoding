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

### E — Cellular Automata & Grid Simulations
- [E1 Falling Sand — Gravity CA](#e1-falling-sand--gravity-ca)
- [E2 Doom-style Fire — Heat Diffusion CA](#e2-doom-style-fire--heat-diffusion-ca)
- [E3 aafire 5-Neighbour CA](#e3-aafire-5-neighbour-ca)
- [E4 Processing Order & Artefact Suppression](#e4-processing-order--artefact-suppression)
- [E5 Stochastic Rules](#e5-stochastic-rules)

### F — Noise & Procedural Generation
- [F1 Perlin Noise — Permutation Table & Smoothstep](#f1-perlin-noise--permutation-table--smoothstep)
- [F2 Octave Layering (Fractal Brownian Motion)](#f2-octave-layering-fractal-brownian-motion)
- [F3 Flow Field from Noise](#f3-flow-field-from-noise)
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

### K — Shading Models
- [K1 Blinn-Phong Shading](#k1-blinn-phong-shading)
- [K2 Toon / Cel Shading — Banded Diffuse](#k2-toon--cel-shading--banded-diffuse)
- [K3 Normal Visualisation Shader](#k3-normal-visualisation-shader)
- [K4 Parametric Torus Lighting (Donut)](#k4-parametric-torus-lighting-donut)

### L — Algorithms & Data Structures
- [L1 Bresenham Line Algorithm](#l1-bresenham-line-algorithm)
- [L2 Ring Buffer](#l2-ring-buffer)
- [L3 Z-buffer / Depth Sort](#l3-z-buffer--depth-sort)
- [L4 Bounding Box Rasterization](#l4-bounding-box-rasterization)
- [L5 Fisher-Yates Shuffle](#l5-fisher-yates-shuffle)
- [L6 Callback / Function Pointer Patterns](#l6-callback--function-pointer-patterns)
- [L7 Lookup Table (LUT)](#l7-lookup-table-lut)
- [L8 Bilinear Interpolation](#l8-bilinear-interpolation)

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

### Q — Artistic / Signal-Based Effects
- [Q1 Sinusoidal Aurora Curtains](#q1-sinusoidal-aurora-curtains)
- [Q2 Demoscene Plasma — Sin-Sum Palette Cycling](#q2-demoscene-plasma--sin-sum-palette-cycling)
- [Q3 Hypotrochoid Spirograph with Float Canvas Decay](#q3-hypotrochoid-spirograph-with-float-canvas-decay)
- [Q4 Voronoi with Langevin Brownian Seeds](#q4-voronoi-with-langevin-brownian-seeds)

### R — Force-Based Physics
- [R1 Explicit Spring Forces + Symplectic Euler (Cloth)](#r1-explicit-spring-forces--symplectic-euler-cloth)
- [R2 Softened N-Body Gravity + Verlet](#r2-softened-n-body-gravity--verlet)
- [R3 Lorenz Strange Attractor — RK4 + 3-D Projection](#r3-lorenz-strange-attractor--rk4--3-d-projection)

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

#### E2 Doom-style Fire — Heat Diffusion CA
Each cell's heat value diffuses upward by averaging with three neighbours below, then subtracts a small decay. The bottom row is periodically seeded with maximum heat. The result is a convincing fire that rises, flickers, and fades — achieved with a 3-line update rule and no fluid simulation.
*Files: `fire.c`*

#### E3 aafire 5-Neighbour CA
The aalib variant samples five neighbours (three below plus two diagonals two rows below) and averages them, producing rounder, slower-rising blobs compared to Doom's sharper spikes. A precomputed `minus` value based on screen height normalises the decay rate so the flame height is consistent at any terminal size.
*Files: `aafire_port.c`*

#### E4 Processing Order & Artefact Suppression
Scanning top-to-bottom in a falling CA lets grains move multiple cells in a single pass — they "teleport." Scanning bottom-to-top fixes this. For horizontal neighbours, randomising the left/right scan order each row removes the diagonal bias that otherwise makes all sand pile up on one side.
*Files: `sand.c`*

#### E5 Stochastic Rules
Adding `rand() % 2` to decide which diagonal direction to try, or whether to scatter a fire cell, gives organic variation with almost no code. Deterministic rules produce repetitive, crystalline patterns; a single random branch breaks the symmetry and makes the simulation look alive.
*Files: `sand.c`, `fire.c`, `aafire_port.c`, `flowfield.c`*

---

### F — Noise & Procedural Generation

#### F1 Perlin Noise — Permutation Table & Smoothstep
Ken Perlin's classic algorithm hashes integer grid corners using a 256-element permutation table, then blends the four corner contributions using a smoothstep curve (`6t⁵ - 15t⁴ + 10t³`). The result is a continuous, band-limited noise signal that looks natural — unlike `rand()` which has no spatial coherence.
*Files: `flowfield.c`*

#### F2 Octave Layering (Fractal Brownian Motion)
Summing multiple noise samples at increasing frequencies (`freq × 2ⁿ`) and decreasing amplitudes (`amp × 0.5ⁿ`) builds fractal detail. Two octaves give smooth hills; four give terrain with boulders; eight give bark texture. This project uses three octaves for the flow field angle, balancing detail against computation.
*Files: `flowfield.c`*

#### F3 Flow Field from Noise
Sample two independent noise fields at offset coordinates to get `(vx, vy)`, then `atan2(vy, vx)` gives an angle. Placing this angle at every grid cell builds a vector field that is spatially smooth but visually complex. Particles that follow the field produce curved, organic-looking trails.
*Files: `flowfield.c`*

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
Display hardware applies a nonlinear transfer function (gamma ≈ 2.2) to the stored color value. Working in linear light (as Phong shading does) and outputting without correction makes the image look too dark. Applying `pow(value, 1/2.2)` before output converts linear light to gamma-encoded display values and restores the correct perceptual brightness.
*Files: all raster files, `raymarcher.c`*

#### G6 Directional Characters — Arrow & Line Glyphs
In `flowfield.c` the particle head character is chosen by the angle of motion: `→ ↗ ↑ ↖ ← ↙ ↓ ↘`. Dividing `atan2(vy,vx)` by `π/4` and rounding to the nearest octant indexes into an 8-character array. In `spring_pendulum.c` the spring is drawn with `/`, `\`, `|`, `-` chosen by the local segment slope.
*Files: `flowfield.c`, `spring_pendulum.c`, `wireframe.c`*

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
*Files: `matrix_rain.c`, `flowfield.c`*

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
Sampling a 2D grid at a non-integer position by weighting the four surrounding grid cells: `lerp(lerp(top-left, top-right, fx), lerp(bottom-left, bottom-right, fx), fy)`. `flowfield.c` uses bilinear interpolation to sample the noise field between grid points, producing a smooth continuous velocity field rather than a blocky step function.
*Files: `flowfield.c`*

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
*Files: all raster files, `flowfield.c`, `sand.c`*

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

*Read the code, run the programs, change one constant at a time. That is how it becomes yours.*
