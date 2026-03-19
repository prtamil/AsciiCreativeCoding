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

*Read the code, run the programs, change one constant at a time. That is how it becomes yours.*
