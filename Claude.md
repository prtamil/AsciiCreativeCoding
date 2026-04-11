# Terminal Demos — ncurses C Projects

## Build

```bash
# ── basics ───────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/tst_lines_cols.c  -o tst_lines_cols  -lncurses
gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/aspect_ratio.c    -o aspect_ratio    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/spring_pendulum.c -o spring_pendulum -lncurses -lm

# ── misc ─────────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra misc/bounce_ball.c       -o bounce          -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra misc/bonsai.c            -o bonsai          -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra misc/double_pendulum.c   -o double_pendulum -lncurses -lm

# ── matrix ───────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra matrix_rain/matrix_rain.c -o matrix_rain -lncurses -lm

# ── particle systems ─────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra particle_systems/fire.c         -o fire         -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/aafire_port.c  -o aafire        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/fireworks.c    -o fireworks     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/brust.c        -o brust         -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/kaboom.c       -o kaboom        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra particle_systems/constellation.c -o constellation -lncurses -lm

# ── flocking ─────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra flocking/flocking.c             -o flocking      -lncurses -lm

# ── fluid / grid sims ────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra fluid/sand.c                    -o sand                -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/flowfield.c               -o flowfield           -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/reaction_diffusion.c      -o reaction_diffusion  -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fluid/wave.c                    -o wave                -lncurses -lm

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
gcc -std=c11 -O2 -Wall -Wextra physics/lorenz.c             -o lorenz       -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/nbody.c              -o nbody        -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra physics/cloth.c              -o cloth        -lncurses -lm

# ── fractal / random growth (new) ────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra fractal_random/penrose.c     -o penrose      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra fractal_random/terrain.c     -o terrain      -lncurses -lm

# ── artistic ──────────────────────────────────────────────────────────────
gcc -std=c11 -O2 -Wall -Wextra Artistic/bat.c               -o bat          -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra Artistic/2stroke.c           -o 2stroke      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/leaf_fall.c         -o leaf_fall    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/epicycles.c         -o epicycles    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/string_art.c        -o string_art          -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/cellular_automata_1d.c -o cellular_automata_1d -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/life.c                -o life                -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/langton.c             -o langton             -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/cymatics.c            -o cymatics            -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/wator.c               -o wator               -lncurses
gcc -std=c11 -O2 -Wall -Wextra artistic/lissajous.c           -o lissajous           -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/aurora.c              -o aurora              -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/voronoi.c             -o voronoi             -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/spirograph.c          -o spirograph          -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra artistic/plasma.c              -o plasma              -lncurses -lm

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
- `bounce_ball.c`       — **reference implementation** — bouncing balls with pixel-space physics, fixed-timestep accumulator, render interpolation (alpha), SIGWINCH resize
- `bonsai.c`            — growing bonsai tree: recursive branch growth, 5 tree types, pot styles, message panel with ACS box-drawing chars, `use_default_colors`
- `double_pendulum.c`   — double pendulum with RK4 integration demonstrating chaos: ghost pendulum (θ₁+ε), 500-slot trail ring buffer, divergence HUD metric; COMPRESSION/POWER/EXHAUST phases

### matrix_rain/
- `matrix_rain.c`       — Matrix-style falling character rain: two-pass draw, theme system, render interpolation for smooth column-head scrolling

### particle_systems/
- `fire.c`              — Doom-style fire CA: heat diffusion, Floyd-Steinberg dithering, 6 auto-cycling color themes, wind and gravity controls
- `aafire_port.c`       — aalib fire variant: 5-neighbour CA, per-row decay LUT, 9-step `attr_t` brightness gradient
- `fireworks.c`         — rocket fireworks: IDLE → RISING → EXPLODED state machine, particle pool, gravity + drag
- `brust.c`             — random explosion bursts: staggered particle waves, scorch mark persistence, `A_DIM` residue rendering
- `kaboom.c`            — deterministic LCG explosions: same seed → same explosion shape, color-ring blast zones
- `constellation.c`     — star constellation: Bresenham stippled lines with `cell_used[][]` dedup, proximity `A_BOLD`, `prev/cur` lerp interpolation

### flocking/
- `flocking.c`          — boid flocking: 3 flock groups, 5 switchable modes (classic boids, leader chase, Vicsek, orbit, predator-prey), toroidal wrap, cosine palette color cycling, `A_BOLD` proximity halo

### fluid/
- `sand.c`                  — falling sand CA
- `reaction_diffusion.c`   — Gray-Scott model: 7 presets (Mitosis/Coral/Stripes/Worms/Maze/Bubbles/Solitons), 9-point isotropic Laplacian, 4 colour themes (ocean/forest/magma/violet), 600-step warmup pre-run, auto-cycle theme
- `wave.c`                 — FDTD 2-D wave equation: 5 oscillating sources (keys 1–5 toggle), Gaussian impulse (p), interference fringes + boundary reflections, 9-level signed amplitude display, 4 colour themes (water/lava/plasma/matrix), CFL-stable c=0.45
- `flowfield.c`         — Perlin noise flow field: 3-octave fBm, bilinear field sampling, 8-direction arrow glyphs, ring-buffer particle trails

### physics/
- `lorenz.c`       — Lorenz strange attractor: RK4 integration, rotating orthographic 3-D→2-D projection, 1500-slot ring-buffer trail (red→grey), ghost trajectory showing Lyapunov chaos divergence
- `nbody.c`        — N-body gravity: 20 point masses, softened 1/r² (ε=4 px), Velocity Verlet, per-body 200-slot color-faded trails, optional central black hole
- `cloth.c`        — Spring-mass cloth: explicit spring forces (Hooke + velocity damping), symplectic Euler, 3 modes (hanging/flag/hammock), render lerp for smooth sub-tick motion

### fractal_random/
- `penrose.c`      — Penrose P3 rhombus tiling: de Bruijn pentagrid duality, O(1) per cell, parity-based thick/thin distinction, pentagrid edge detection for visible tile outlines (|/\-), slow rotation, 256-color warm/cool palette
- `terrain.c`      — Diamond-square fractal terrain: 65×65 heightmap, ROUGHNESS=0.60, thermal weathering erosion (TALUS=0.022), bilinear interpolation to any terminal size, 7 ASCII contour levels (~.−^#*)
- `snowflake.c`    — DLA crystal with D6 6-fold symmetry; 12-way simultaneous freeze; distance-based 6-color ice palette (light-blue core → teal → white tips); context-sensitive chars (`*` `|` `-` `+` `/` `\`)
- `coral.c`        — anisotropic DLA: 8 bottom seeds, top-spawned walkers with downward bias; direction-dependent sticking probability; vivid coral/violet/yellow/lime/teal palette; auto-reset when tallest branch hits top quarter
- `sierpinski.c`   — Sierpinski triangle via chaos game (IFS): 3 vertices, random vertex → move halfway; N_PER_TICK=500, TOTAL_ITERS=50000; color by last chosen vertex (cyan/yellow/magenta); held then reset
- `fern.c`         — Barnsley Fern: 4-transform IFS (stem 1%, main 85%, left 7%, right 7%); N_PER_TICK=400, TOTAL_ITERS=80000; independent x/y scale to correct aspect; green gradient palette
- `julia.c`        — Julia set with Fisher-Yates random pixel reveal; 6 presets cycling (rabbit, spiral, dendrite, flame, seahorse, basilica); fire palette (white→yellow→orange→red); PIXELS_PER_TICK=60, MAX_ITER=128
- `mandelbrot.c`   — Mandelbrot set (z₀=0, z→z²+c); same Fisher-Yates fill as julia.c; 6 zoom presets including deep spirals; electric neon palette (magenta/purple/cyan/lime/yellow); MAX_ITER=256
- `koch.c`         — Koch snowflake: recursive midpoint subdivision; levels 1–5 cycle; Bresenham rasterization; adaptive segs_per_tick for ~2 s per level; 5-color vivid gradient (cyan→teal→lime→yellow→white)
- `lightning.c`    — fractal branching lightning: recursive tip branching (not DLA); tips grow downward with persistent lean bias, fork after MIN_FORK_STEPS; glow halo radius 2; color by depth (light-blue → teal → white); state machine ST_GROWING → ST_STRIKING → ST_FADING
- `buddhabrot.c`   — Buddhabrot density accumulator: two-pass orbit sampling (escape test then trace); 5 presets (buddha 500/2000, anti 100/500/1000); log-normalized density→color (mode-aware floor: 0.05 buddha / 0.25 anti); purple→white nebula palette

### Artistic/
- `bat.c`          — ASCII bat swarms in Pascal-triangle formation: 3 groups × (n_rows+1)(n_rows+2)/2 bats each; `+`/`-` resize rows 1–6 live; wing animation `/−\−`; 6 preset launch angles; staggered group launch; light-purple/cyan/pink groups
- `2stroke.c`      — 2-stroke engine cross-section: slider-crank kinematics (piston/rod/crankshaft), exhaust and transfer port open/close, spark at TDC, phase labels COMPRESSION/IGNITION/POWER/EXHAUST/SCAVENGING; `] [` RPM control
- `leaf_fall.c`    — ASCII tree with matrix-rain leaf fall
- `epicycles.c`    — Fourier epicycles: DFT of parametric shape (heart/star/trefoil/fig-8/butterfly), sorted-by-amplitude arm chain, auto-adds epicycles to show convergence; orbit circles, colour-faded trail
- `string_art.c`  — Mathematical string art: N=200 nails on circle, threads connect i→round(i×k) mod N, k drifts 2→7.5 morphing cardioid/nephroid/deltoid/astroid; rainbow thread colors (12-step palette), visible nail markers (o), circle rim (·), [/] nail count presets 6–200
- `cellular_automata_1d.c` — Wolfram 1-D elementary CA: 17 preset rules (30/90/110/18/150…), builds top-down row-by-row, 5 Wolfram classes color-coded, auto-cycle, type any rule 0–255+Enter
- `life.c`    — Conway's Game of Life + 5 rule variants (HighLife/Day&Night/Seeds/Morley/2×2), 6-colour palette, Gosper gun/glider/acorn/R-pentomino seeds, 3-row scrolling population histogram
- `langton.c` — Langton's Ant + 7 multi-colour turmite rules (RL/LR/LLRR/RLR/LRRL…), 1–3 simultaneous ants, cell states colour-coded, ant shown as bold @
- `cymatics.c`  — Chladni figures: Z=cos(mπx)cos(nπy)−cos(nπx)cos(mπy), 20 (m,n) modes, nodal lines rendered with glow chars (@#*+.), smooth morph animation between modes, 4 colour themes
- `aurora.c`    — Aurora borealis: two-octave sinusoidal curtains with vertical sine envelope, color zones by row fraction (magenta fringe/cyan core/green base), deterministic star hash (no storage, no flicker)
- `voronoi.c`   — Animated Voronoi: 24 seeds with Langevin Brownian motion (DAMP=2, NOISE=60), brute-force O(N) nearest-seed per cell, d2−d1 edge detection, seed bounce off walls
- `spirograph.c` — Spirograph: 3 simultaneous hypotrochoids with parameter drift, float canvas decay (FADE=0.985), 5 intensity levels, cyan/magenta/yellow curves
- `plasma.c`    — Demoscene plasma: 4-component sin-sum (horizontal/vertical/diagonal/radial), palette cycling via time-offset, 4 frequency presets (gentle/energetic/grand/turbulent), 4 color themes, 14 PalEntry levels each

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
