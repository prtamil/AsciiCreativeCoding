# Terminal Animation Framework — Architecture

Reference implementation: `basics/bounce_ball.c`

---

## Table of Contents

1. [Overview](#1-overview)
2. [Coordinate Spaces](#2-coordinate-spaces)
3. [Fixed Timestep with Accumulator](#3-fixed-timestep-with-accumulator)
4. [Render Interpolation (Alpha)](#4-render-interpolation-alpha)
5. [Frame Cap — Sleep Before Render](#5-frame-cap--sleep-before-render)
6. [ncurses Double Buffer — How It Actually Works](#6-ncurses-double-buffer--how-it-actually-works)
7. [ncurses Optimisations](#7-ncurses-optimisations)
8. [Section Breakdown](#8-section-breakdown)
   - [§1 config](#1-config)
   - [§2 clock](#2-clock)
   - [§3 color](#3-color)
   - [§4 coords](#4-coords)
   - [§5 physics (ball / pendulum / particle …)](#5-physics-ball--pendulum--particle-)
   - [§6 scene](#6-scene)
   - [§7 screen](#7-screen)
   - [§8 app](#8-app)
9. [Main Loop — Annotated Walk-through](#9-main-loop--annotated-walk-through)
10. [Signal Handling and Cleanup](#10-signal-handling-and-cleanup)
11. [Adding a New Animation](#11-adding-a-new-animation)
12. [Software Rasterizer — raster/*.c](#12-software-rasterizer--rasterc)
    - [Pipeline Overview](#pipeline-overview)
    - [ShaderProgram and the Split Uniform Fix](#shaderprogram-and-the-split-uniform-fix)
    - [Framebuffer](#framebuffer)
    - [Mesh and Tessellation](#mesh-and-tessellation)
    - [Vertex and Fragment Shaders](#vertex-and-fragment-shaders)
    - [Displacement (displace_raster.c)](#displacement-displace_rasterc)
13. [Cellular Automata & Particle Grid Sims — fire.c, smoke.c, aafire_port.c, sand.c](#13-cellular-automata--firec-smokec-aafire_portc-sandc)
14. [Flow Field — flowfield.c, complex_flowfield.c](#14-flow-field--flowfieldc-complex_flowfieldc)
15. [Flocking Simulation — flocking.c](#15-flocking-simulation--flockingc)
16. [Bonsai Tree — bonsai.c](#16-bonsai-tree--bonsaic)
17. [Constellation — constellation.c](#17-constellation--constellationc)
18. [Raymarchers and Donut — raymarcher/*.c](#18-raymarchers-and-donut--raymarcherc)
    - [donut.c — Parametric Torus](#donutc--parametric-torus-no-mesh-no-sdf)
    - [wireframe.c — 3-D Projected Edges](#wireframec--3-d-projected-edges)
    - [SDF Sphere-Marcher Pipeline](#sdf-sphere-marcher-pipeline)
    - [Primitive Composition](#primitive-composition-raymarcher_primitivesc)
19. [Particle State Machines — fireworks.c, brust.c, kaboom.c](#19-particle-state-machines--fireworksc-brustc-kaboomc)
    - [fireworks.c — Three-State Rocket](#fireworksc--three-state-rocket)
    - [brust.c — Staggered Burst Waves](#brustc--staggered-burst-waves)
    - [kaboom.c — Deterministic LCG Explosions](#kaboomc--deterministic-lcg-explosions)
20. [Matrix Rain — matrix_rain.c](#20-matrix-rain--matrix_rainc)
20b. [Matrix Snowflake — matrix_snowflake.c](#20b-matrix-snowflake--matrix_snowflakec)
21. [Documentation Files Reference](#21-documentation-files-reference)
22. [Fractal / Random Growth — fractal_random/](#22-fractal--random-growth--fractal_random)
    - [DLA — snowflake.c and coral.c](#dla--snowflakec-and-coralc)
    - [D6 Hexagonal Symmetry with Aspect Correction](#d6-hexagonal-symmetry-with-aspect-correction)
    - [Anisotropic DLA — coral.c](#anisotropic-dla--coralc)
    - [Chaos Game / IFS — sierpinski.c and fern.c](#chaos-game--ifs--sierpinskic-and-fernc)
    - [Barnsley Fern IFS Transforms](#barnsley-fern-ifs-transforms)
    - [Escape-time Sets — julia.c and mandelbrot.c](#escape-time-sets--juliac-and-mandelbrotc)
    - [Fisher-Yates Random Pixel Reveal](#fisher-yates-random-pixel-reveal)
    - [Koch Snowflake — koch.c](#koch-snowflake--kochc)
    - [Recursive Tip Branching — lightning.c](#recursive-tip-branching--lightningc)
37. [Physics Simulations — physics/](#37-physics-simulations--physics)
    - [Lorenz Strange Attractor — lorenz.c](#lorenz-strange-attractor--lorenzc)
    - [N-Body Gravity — nbody.c](#n-body-gravity--nbodyc)
    - [Spring-Mass Cloth — cloth.c](#spring-mass-cloth--clothc)
    - [Magnetic Field Lines — magnetic_field.c](#magnetic-field-lines--magnetic_fieldc)
    - [Hanging Chain — chain.c](#hanging-chain--chainc)
    - [Slime Mold — slime_mold.c](#slime-mold--slime_moldc)
    - [Beam Bending & Vibration — beam_bending.c](#beam-bending--vibration--beam_bendingc)
    - [Differential Drive Robot — diff_drive_robot.c](#differential-drive-robot--diff_drive_robotc)
38. [Artistic Effects — artistic/ & geometry/](#38-artistic-effects--artistic)
    - [Aurora Borealis — aurora.c](#aurora-borealis--aurorac)
    - [Animated Voronoi — voronoi.c](#animated-voronoi--voronoic)
    - [Spirograph — spirograph.c](#spirograph--spirographc)
    - [Plasma — plasma.c](#plasma--plasmac)
39. [Penrose Tiling — fractal_random/penrose.c](#39-penrose-tiling--fractal_randompenrosec)
40. [Diamond-Square Terrain — fractal_random/terrain.c](#40-diamond-square-terrain--fractal_randomterrainc)
41. [Forest Fire CA — misc/forest_fire.c](#41-forest-fire-ca--miscforest_firec)
42. [Lattice Gas — fluid/lattice_gas.c](#42-lattice-gas--fluidlattice_gasc)
43. [Galaxy — artistic/galaxy.c](#43-galaxy--artisticgalaxyc)
44. [Barnes-Hut — physics/barnes_hut.c](#44-barnes-hut--physicsbarnes_hutc)
45. [Analytic Wave Interference — fluid/wave_interference.c](#45-analytic-wave-interference--fluidwave_interferencec)
46. [7-Segment LED Morph — artistic/led_number_morph.c](#46-7-segment-led-morph--artisticled_number_morphc)
47. [Particle Number Morph — artistic/particle_number_morph.c](#47-particle-number-morph--artisticparticle_number_morphc)
48. [Julia Explorer — fractal_random/julia_explorer.c](#48-julia-explorer--fractal_randomjulia_explorerc)
49. [IFS Chaos Game — fractal_random/barnsley.c](#49-ifs-chaos-game--fractal_randombarnsleyc)
50. [DLA Extended — fractal_random/diffusion_map.c](#50-dla-extended--fractal_randomdiffusion_mapc)
51. [DBM Tree — fractal_random/tree_la.c](#51-dbm-tree--fractal_randomtree_lac)
52. [Lyapunov Fractal — fractal_random/lyapunov.c](#52-lyapunov-fractal--fractal_randomlyapunovc)
53. [Ant Colony Optimization — artistic/ant_colony.c](#53-ant-colony-optimization--artisticant_colonyc)
54. [Dune Rocket — artistic/dune_rocket.c](#54-dune-rocket--artisticdune_rocketc)
55. [Dune Sandworm — artistic/dune_sandworm.c](#55-dune-sandworm--artisticdune_sandwormc)
56. [FFT Visualiser — artistic/fft_vis.c](#56-fft-visualiser--artisticfft_visc)
57. [Fourier Draw — artistic/fourier_draw.c](#57-fourier-draw--artisticfourier_drawc)
58. [Gear — artistic/gear.c](#58-gear--artisticgearc)
59. [Graph Search — artistic/graph_search.c](#59-graph-search--artisticgraph_searchc)
60. [Hex Life — artistic/hex_life.c](#60-hex-life--artistichex_lifec)
61. [Jellyfish — artistic/jellyfish.c](#61-jellyfish--artisticjellyfish)
62. [Network SIR Simulation — artistic/network_sim.c](#62-network-sir-simulation--artisticnetwork_simc)
63. [Railway Map — artistic/railwaymap.c](#63-railway-map--artisticrailwaymapc)
64. [Sand Art — artistic/sand_art.c](#64-sand-art--artisticsand_artc)
65. [X-Ray Swarm — artistic/xrayswarm.c](#65-x-ray-swarm--artisticxrayswarm)
66. [Shepherd Herding — flocking/shepherd.c](#66-shepherd-herding--flockingshepherdc)
67. [Excitable Medium — fluid/excitable.c](#67-excitable-medium--fluidexcitablec)
68. [SPH Fluid — fluid/fluid_sph.c](#68-sph-fluid--fluidfluid_sphc)
69. [Lenia — fluid/lenia.c](#69-lenia--fluidleniac)
70. [Marching Squares — fluid/marching_squares.c](#70-marching-squares--fluidmarching_squaresc)
71. [Stable Fluids (Navier-Stokes) — fluid/navier_stokes.c](#71-stable-fluids-navier-stokes--fluidnavier_stokesc)
72. [FitzHugh-Nagumo Reaction Wave — fluid/reaction_wave.c](#72-fitzhugh-nagumo-reaction-wave--fluidreaction_wavec)
73. [2-D Wave Equation — fluid/wave_2d.c](#73-2-d-wave-equation--fluidwave_2dc)
74. [Apollonian Gasket — fractal_random/apollonian.c](#74-apollonian-gasket--fractal_randomapolonianc)
75. [2-D Cellular Automaton (LtL) — fractal_random/automaton_2d.c](#75-2-d-cellular-automaton-ltl--fractal_randomautomaton_2dc)
76. [Bifurcation Diagram — fractal_random/bifurcation.c](#76-bifurcation-diagram--fractal_randombifurcationc)
77. [Burning Ship Fractal — fractal_random/burning_ship.c](#77-burning-ship-fractal--fractal_randomburning_shipc)
78. [Dragon Curve — fractal_random/dragon_curve.c](#78-dragon-curve--fractal_randomdragon_curvec)
79. [L-System — fractal_random/l_system.c](#79-l-system--fractal_randoml_systemc)
80. [Newton Fractal — fractal_random/newton_fractal.c](#80-newton-fractal--fractal_randomnewton_fractalc)
81. [Perlin Landscape — fractal_random/perlin_landscape.c](#81-perlin-landscape--fractal_randomperlin_landscapec)
82. [Strange Attractor — fractal_random/strange_attractor.c](#82-strange-attractor--fractal_randomstrange_attractorc)
83. [Convex Hull — geometry/convex_hull.c](#83-convex-hull--geometryconvex_hullc)
84. [Hex Grid — geometry/hex_grid.c](#84-hex-grid--geometryhex_gridc)
85. [Polar Grid — geometry/polar_grid.c](#85-polar-grid--geometrypolar_gridc)
86. [Rect Grid — geometry/rect_grid.c](#86-rect-grid--geometryrect_gridc)
87. [Fireworks Rain — matrix_rain/fireworks_rain.c](#87-fireworks-rain--matrix_rainfireworks_rainc)
88. [Pulsar Rain — matrix_rain/pulsar_rain.c](#88-pulsar-rain--matrix_rainpulsar_rainc)
89. [Sun Rain — matrix_rain/sun_rain.c](#89-sun-rain--matrix_rainsun_rainc)
90. [Maze — misc/maze.c](#90-maze--miscmazec)
91. [Sort Visualiser — misc/sort_vis.c](#91-sort-visualiser--miscsort_visc)
92. [Aspect Ratio Demo — ncurses_basics/aspect_ratio.c](#92-aspect-ratio-demo--ncurses_basicsaspect_ratioc)
93. [Lines and Cols Query — ncurses_basics/tst_lines_cols.c](#93-lines-and-cols-query--ncurses_basicstst_lines_colsc)
94. [2-Stroke Engine — physics/2stroke.c](#94-2-stroke-engine--physics2strokec)
95. [Black Hole — physics/blackhole.c](#95-black-hole--physicsblackholec)
96. [Bubble Chamber — physics/bubble_chamber.c](#96-bubble-chamber--physicsbubble_chamberc)
97. [Elastic Collision — physics/elastic_collision.c](#97-elastic-collision--physicselastic_collisionc)
98. [Gyroscope — physics/gyroscope.c](#98-gyroscope--physicsgyroscopec)
99. [Three-Body Orbit — physics/orbit_3body.c](#99-three-body-orbit--physicsorbit_3bodyc)
100. [Pendulum Wave — physics/pendulum_wave.c](#100-pendulum-wave--physicspendulum_wavec)
101. [Rigid Body — physics/rigid_body.c](#101-rigid-body--physicsrigid_bodyc)
102. [Schrödinger Equation — physics/schrodinger.c](#102-schrödinger-equation--physicsschrodingerc)
103. [Soft Body (PBD) — physics/soft_body.c](#103-soft-body-pbd--physicssoft_bodyc)
104. [Mandelbulb Rasterizer — raster/mandelbulb_raster.c](#104-mandelbulb-rasterizer--rastermandelbulb_rasterc)
105. [Mandelbulb Explorer — raymarcher/mandelbulb_explorer.c](#105-mandelbulb-explorer--raymarchermandelbulb_explorerc)
106. [SDF Gallery — raymarcher/sdf_gallery.c](#106-sdf-gallery--raymarchersdf_galleryc)
107. [Capsule Raytrace — raytracing/capsule_raytrace.c](#107-capsule-raytrace--raytracingtcapsule_raytracec)
108. [Cube Raytrace — raytracing/cube_raytrace.c](#108-cube-raytrace--raytracingcube_raytracec)
109. [Path Tracer — raytracing/path_tracer.c](#109-path-tracer--raytracingpath_tracerc)
110. [Sphere Raytrace — raytracing/sphere_raytrace.c](#110-sphere-raytrace--raytracing-sphere_raytracec)
111. [Torus Raytrace — raytracing/torus_raytrace.c](#111-torus-raytrace--raytracingtorus_raytracec)
112. [Beam Bending & Vibration — physics/beam_bending.c](#112-beam-bending--vibration--physicsbeam_bendingc)
113. [Differential Drive Robot — robots/diff_drive_robot.c](#113-differential-drive-robot--physicsdiff_drive_robotc)
114. [Walking Robot — robots/walking_robot.c](#114-walking-robot--robotswalking_robotc)
115. [Perlin Terrain Bot — robots/perlin_terrain_bot.c](#115-perlin-terrain-bot--robotsperlin_terrain_botc)

---

## 1. Overview

Every animation in this project follows the same layered loop:

```
┌─────────────────────────────────────────────────────────┐
│                        main loop                        │
│                                                         │
│  ① measure dt (wall-clock elapsed since last frame)     │
│  ② drain sim accumulator → fixed-step physics ticks     │
│  ③ compute alpha (sub-tick render offset)               │
│  ④ sleep to cap output at 60 fps (BEFORE render)        │
│  ⑤ draw frame at interpolated position → stdscr         │
│  ⑥ doupdate() → one diff write to terminal              │
│  ⑦ poll input                                           │
└─────────────────────────────────────────────────────────┘
```

The design separates three concerns:

| Concern | Where | Rate |
|---|---|---|
| Physics simulation | `scene_tick()` | Fixed (e.g. 60 or 120 Hz) |
| Rendering | `screen_draw()` + `doupdate()` | Capped at 60 fps |
| Input | `getch()` | Every frame, non-blocking |

Physics and rendering are deliberately decoupled. The sim can run at 120 Hz for accuracy while the display stays at 60 fps, or the sim can run at 24 Hz for a stylised effect while still rendering smoothly.

---

## 2. Coordinate Spaces

**The root problem with naive terminal animation:**

Terminal cells are not square. A typical cell is roughly twice as tall as it is wide in physical pixels (e.g. 8 px wide × 16 px tall). If you store a ball's position directly in cell coordinates and move it by `dx = 1, dy = 1` per tick, it travels twice as far horizontally as vertically in physical pixels. Diagonal motion looks skewed. Circles become ellipses.

**The fix — two coordinate spaces, one conversion point:**

```
PIXEL SPACE          (physics lives here)
─────────────────────────────────────────
• Square grid. One unit ≈ one physical pixel.
• Width  = cols × CELL_W   (e.g. 200 cols × 8  = 1600 px)
• Height = rows × CELL_H   (e.g.  50 rows × 16 =  800 px)
• All positions, velocities, forces are in pixel units.
• Speed is isotropic — 1 px/s covers equal physical distance in X and Y.

CELL SPACE           (drawing happens here)
─────────────────────────────────────────
• Terminal columns and rows.
• cell_x = px_to_cell_x(pixel_x)
• cell_y = px_to_cell_y(pixel_y)
• Only scene_draw() ever calls px_to_cell_x/y.
• Physics code never sees cell coordinates.
```

```c
/* bounce_ball.c §4 */
#define CELL_W   8
#define CELL_H  16

static inline int pw(int cols) { return cols * CELL_W; }  /* pixel width  */
static inline int ph(int rows) { return rows * CELL_H; }  /* pixel height */

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}
```

**Why `floorf(px/CELL_W + 0.5f)` instead of `roundf` or truncation:**

- `roundf` uses "round half to even" (banker's rounding). When `px/CELL_W` is exactly `0.5` it can round to 0 on one call and 1 on the next depending on FPU state. A ball sitting on a cell boundary oscillates between two cells every frame — visible flicker.
- Truncation `(int)(px/CELL_W)` always rounds down. Creates asymmetric dwell time — staircase effect.
- `floorf(x + 0.5f)` is "round half up" — always deterministic, breaks ties in the same direction, symmetric dwell time with no oscillation.

**Simulations that don't need two spaces** (sand.c, fire.c, flowfield.c) work directly in cell coordinates because cells themselves are the physics grid. Those files omit §4 entirely.

---

## 3. Fixed Timestep with Accumulator

**Why fixed timestep:**

A variable timestep simulation (where `dt` passed to `tick()` equals whatever the wall clock measured) produces physically incorrect results. If a frame takes twice as long as usual, the tick receives `2×dt` and objects overshoot. Springs explode, collisions are missed, anything that depends on a maximum step size breaks. Floating-point integration errors also grow non-linearly with `dt`.

**The accumulator pattern:**

```c
/* bounce_ball.c §8 main loop */
int64_t tick_ns = TICK_NS(app->sim_fps);   /* e.g. 1/60 s = 16,666,666 ns */
float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

sim_accum += dt;                           /* dt = measured wall-clock ns   */
while (sim_accum >= tick_ns) {
    scene_tick(&app->scene, dt_sec, cols, rows);
    sim_accum -= tick_ns;
}
```

How it works:

1. `sim_accum` is a nanosecond bucket.
2. Every frame, the measured wall-clock `dt` is added to the bucket.
3. While the bucket holds enough for a full tick, one fixed-size physics step is consumed.
4. The remainder stays in the bucket for next frame.

```
Frame 1:  dt = 18 ms   sim_accum = 18 ms   → 1 tick (16.7 ms), leftover = 1.3 ms
Frame 2:  dt = 15 ms   sim_accum = 16.3 ms → 1 tick,           leftover = 0.3 ms  (dropped a tick vs naive)
Frame 3:  dt = 20 ms   sim_accum = 20.3 ms → 1 tick,           leftover = 3.6 ms
Frame 4:  dt =  5 ms   sim_accum =  8.6 ms → 0 ticks           leftover = 8.6 ms  (frame was fast, no tick)
```

Physics runs at exactly `sim_fps` steps per second on average, regardless of render frame rate. The simulation is deterministic, stable, and numerically identical on any machine.

**The dt cap:**

```c
if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;
```

If the process is paused (debugger, suspend, sleep) and then resumed, `dt` would be huge and the accumulator would drain thousands of ticks in one frame, causing an apparent physics jump. The cap clamps to 100 ms maximum — the simulation just "pauses" rather than catching up.

---

## 4. Render Interpolation (Alpha)

**The problem alpha solves:**

After draining the accumulator, `sim_accum` still holds the leftover nanoseconds — the time elapsed into the *next* tick that has not fired yet. If we draw objects at their last ticked position, we are drawing them up to one full tick behind wall-clock "now". At 60 Hz this is a 0–16 ms lag, visible as micro-stutter when the render frame lands just before a tick fires.

**The computation:**

```c
/* bounce_ball.c §8, immediately after the accumulator loop */
float alpha = (float)sim_accum / (float)tick_ns;
```

`alpha` ∈ [0.0, 1.0):

- `0.0` → render fires exactly on a tick boundary; draw position equals physics position.
- `0.9` → render fires 90% of the way through the next tick; draw position is projected 90% of a tick ahead.

**How it is used in `scene_draw`:**

```c
/* bounce_ball.c §6 scene_draw */
float draw_px = b->px + b->vx * alpha * dt_sec;
float draw_py = b->py + b->vy * alpha * dt_sec;
```

Each object's draw position is extrapolated forward by `alpha × dt_sec` seconds from its last ticked position using its current velocity. The drawn position tracks wall-clock "now" to within rendering error.

**Extrapolation vs true interpolation:**

This is technically *forward extrapolation* (predict from current state). True interpolation would store the previous tick's position and lerp between `prev` and `current`. For constant-velocity physics (bounce_ball.c), forward extrapolation is numerically identical to interpolation and requires no extra storage. For non-linear forces (spring_pendulum.c, fireworks.c), use proper lerp:

```c
/* spring_pendulum.c §6 */
float draw_r     = p->prev_r     + (p->r     - p->prev_r)     * alpha;
float draw_theta = p->prev_theta + (p->theta  - p->prev_theta) * alpha;
```

**When the scene is paused:**

`scene_tick` is skipped so physics positions do not change, but `alpha` still advances each frame. The draw position drifts slightly from the frozen physics position. This is imperceptible (less than one cell over the pause duration) and self-corrects when unpaused. To get pixel-perfect freeze, zero `alpha` when paused: `float alpha = scene.paused ? 0.0f : ...`

---

## 5. Frame Cap — Sleep Before Render

**The naive mistake:**

```c
/* WRONG ORDER */
screen_draw(...);          /* terminal I/O — unpredictable duration */
screen_present();          /* doupdate() — more terminal I/O        */
getch();                   /* input poll                            */

/* measure elapsed and sleep */
int64_t elapsed = clock_ns() - frame_time + dt;
clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
```

When the sleep is measured *after* terminal I/O, the elapsed time includes however long `doupdate()` and `getch()` took. On a slow terminal, a large frame might push `elapsed` past the budget entirely, making the sleep zero — the next frame starts immediately, the loop runs full-speed, and the frame rate becomes erratic.

**The correct order:**

```c
/* bounce_ball.c §8 — correct */

/* ① measure elapsed since frame_time (which is now = start of this frame) */
int64_t elapsed = clock_ns() - frame_time + dt;

/* ② sleep the remaining 60fps budget BEFORE any terminal I/O */
clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

/* ③ now do terminal I/O — the sleep has already consumed its budget */
screen_draw(...);
screen_present();
getch();
```

By sleeping first, the measurement captures only physics computation time (cheap, fast, predictable). The terminal I/O still takes variable time, but it is now "free" — it happens after the budget is spent, so it cannot cause the next frame to start late.

**`clock_sleep_ns` implementation:**

```c
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;          /* already over budget — don't sleep */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}
```

The `ns <= 0` guard handles frames where physics was unusually expensive — the sleep is simply skipped rather than causing undefined behaviour with a negative `nanosleep`.

---

## 6. ncurses Double Buffer — How It Actually Works

A common mistake in terminal animation is creating two `WINDOW*` objects and swapping them each frame ("front/back buffer"). This is wrong. ncurses already maintains an internal double buffer:

```
curscr   — what ncurses believes is currently on the physical terminal
newscr   — the frame you are building this render step
```

Every `mvwaddch`, `wattron`, `werase`, `mvprintw` call writes into `newscr`. Nothing reaches the terminal until you call `doupdate()`.

`doupdate()` computes `newscr − curscr` (the diff of changed cells only), sends that minimal set of escape codes to the terminal fd, then updates `curscr = newscr`. This is the double buffer. It is always present. It is not optional.

Adding a manual front/back `WINDOW` pair creates a *third* virtual screen that ncurses does not know about. When you copy from your back window into `stdscr` for display, the diff engine sees spurious changes on every cell, breaking its accuracy and producing ghost trails.

**The correct single-window model:**

```c
/* bounce_ball.c §7 */
erase();                          /* clear newscr — no terminal I/O   */
scene_draw(sc, stdscr, ...);      /* write scene into newscr           */
mvprintw(0, hud_x, "%s", buf);   /* write HUD into newscr (on top)    */
wnoutrefresh(stdscr);             /* copy stdscr into ncurses' newscr  */
doupdate();                       /* diff newscr vs curscr → terminal  */
```

**Properties:**

| Property | Result |
|---|---|
| No flicker | ncurses' diff engine never shows a partial frame |
| No ghost | `curscr` is always accurate — one source of truth |
| No tear | `doupdate()` is one atomic write to the terminal fd |
| HUD Z-order | Written last into same `stdscr` → always on top |

---

## 7. ncurses Optimisations

**`typeahead(-1)`**

```c
typeahead(-1);   /* in screen_init */
```

By default ncurses interrupts its output mid-flush to check whether there is input waiting on stdin. On fast terminals or when many cells change at once, this poll breaks up `doupdate()`'s write into multiple smaller writes, causing visible tearing. `typeahead(-1)` disables the check — ncurses writes the entire diff atomically.

**`nodelay(stdscr, TRUE)`**

```c
nodelay(stdscr, TRUE);
```

Makes `getch()` non-blocking. Without this, `getch()` blocks until a key is pressed, halting the entire loop. With `TRUE`, it returns `ERR` immediately when no key is available.

**`erase()` vs `clear()`**

- `clear()` marks every cell as changed and sends `\e[2J` (clear screen) to the terminal — a full repaint every frame, expensive and flickery.
- `erase()` clears ncurses' `newscr` internal buffer only. No terminal I/O. The diff engine will only send changes, not a full redraw.

Always use `erase()` in the render loop.

**`wnoutrefresh` + `doupdate` vs `wrefresh`**

- `wrefresh(w)` = `wnoutrefresh(w)` + `doupdate()` in one call.
- When you have only one window (`stdscr`), both are equivalent.
- The framework always uses `wnoutrefresh` + `doupdate` explicitly to make the two-phase pattern clear and to allow future multi-window compositing if needed.

**`curs_set(0)`**

Hides the terminal cursor. Without this, the cursor jumps to wherever the last `mvaddch` was called — visible as a flashing dot that moves around the screen every frame.

**`cbreak()` + `noecho()`**

- `cbreak()` delivers keystrokes immediately without waiting for Enter.
- `noecho()` prevents typed characters from being printed to the terminal.

---

## 8. Section Breakdown

Every C file in the project follows the same §-numbered section layout.

### §1 config

All tunable constants in one place. No magic numbers elsewhere in the file.

```c
enum {
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MIN      = 10,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,
    HUD_COLS         = 40,
    FPS_UPDATE_MS    = 500,
    BALLS_DEFAULT    =  5,
    BALLS_MAX        = 20,
    N_COLORS         =  7,
};

#define CELL_W   8
#define CELL_H  16
#define SPEED_MIN  300.0f
#define SPEED_MAX  600.0f
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))
```

`TICK_NS(fps)` converts a frame rate into a tick period in nanoseconds. Used in the accumulator and the frame cap.

### §2 clock

Two functions only: `clock_ns()` and `clock_sleep_ns()`.

```c
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
```

`CLOCK_MONOTONIC` never jumps backward (unlike `CLOCK_REALTIME` which can be adjusted by NTP or the user). Essential for a stable `dt` measurement.

### §3 color

Initialises ncurses color pairs. 256-color path with 8-color fallback.

```c
static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);   /* xterm-256 index */
        ...
    } else {
        init_pair(1, COLOR_RED, COLOR_BLACK);
        ...
    }
}
```

Color pairs are referenced later as `COLOR_PAIR(n)`. `A_BOLD` gives brighter foreground on both 256-color and 8-color terminals.

### §4 coords

Only present in animations that need isotropic physics (bounce_ball.c, spring_pendulum.c). Contains `pw()`, `ph()`, `px_to_cell_x()`, `px_to_cell_y()`. These are the *only* functions in the codebase that cross between pixel and cell space.

Simulations that work in cell coordinates (fire.c, sand.c, matrix_rain.c) do not have this section.

### §5 physics (ball / pendulum / particle …)

The core simulation object and its tick function. Has no knowledge of terminal dimensions, cell coordinates, or ncurses. Receives pixel boundaries from `scene_tick` if needed.

```c
/* bounce_ball.c §5 */
typedef struct {
    float px, py;   /* position in pixel space */
    float vx, vy;   /* velocity in pixel space */
    int   color;
    char  ch;
} Ball;

static void ball_tick(Ball *b, float dt, float max_px, float max_py)
{
    b->px += b->vx * dt;
    b->py += b->vy * dt;
    /* wall bounce */
    if (b->px < 0.0f)   { b->px = 0.0f;   b->vx = -b->vx; }
    if (b->px > max_px) { b->px = max_px;  b->vx = -b->vx; }
    if (b->py < 0.0f)   { b->py = 0.0f;   b->vy = -b->vy; }
    if (b->py > max_py) { b->py = max_py;  b->vy = -b->vy; }
}
```

### §6 scene

Owns the physics object(s) and exposes two functions: `scene_tick` and `scene_draw`.

**`scene_tick`** — advance physics one fixed step. Converts cell dimensions to pixel boundaries once (via `pw`/`ph`) then calls the object's tick function. No ncurses calls.

**`scene_draw`** — draw the current state into `stdscr`. Receives `alpha` and performs the render interpolation. This is the *only* function that calls `px_to_cell_x/y`. Nothing else in the program touches cell coordinates.

```c
/* bounce_ball.c §6 scene_draw (simplified) */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    for (int i = 0; i < s->n; i++) {
        const Ball *b = &s->balls[i];

        /* interpolated draw position — project forward by alpha ticks */
        float draw_px = b->px + b->vx * alpha * dt_sec;
        float draw_py = b->py + b->vy * alpha * dt_sec;

        /* clamp, convert to cell space */
        int cx = px_to_cell_x(draw_px);
        int cy = px_to_cell_y(draw_py);

        wattron(w, COLOR_PAIR(b->color) | A_BOLD);
        mvwaddch(w, cy, cx, (chtype)b->ch);
        wattroff(w, COLOR_PAIR(b->color) | A_BOLD);
    }
}
```

### §7 screen

Owns the ncurses session. Three responsibilities:

1. **`screen_init`** — `initscr`, set all ncurses options, `color_init`, query terminal size.
2. **`screen_draw`** — `erase()`, call `scene_draw`, write HUD. Builds the frame in `newscr`. No terminal I/O.
3. **`screen_present`** — `wnoutrefresh(stdscr)` + `doupdate()`. The single terminal write per frame.
4. **`screen_resize`** — `endwin()` + `refresh()` + re-query size. Called after `SIGWINCH`.

```c
/* bounce_ball.c §7 screen_draw */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  balls:%-2d  %s  spd:%d",
             fps, sc->n, sc->paused ? "PAUSED " : "running", sim_fps);
    int hud_x = s->cols - HUD_COLS;
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}
```

### §8 app

Owns `Scene`, `Screen`, `sim_fps`, and the signal flags. Entry point for everything.

**`App` struct:**

```c
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;
```

`running` and `need_resize` are `volatile sig_atomic_t` because they are written from signal handlers and read from the main loop. `volatile` prevents the compiler from caching the value in a register; `sig_atomic_t` guarantees the read/write is atomic.

**`app_handle_key`** — maps key codes to state changes (pause, add/remove objects, adjust speed). Returns `false` to signal quit.

**`app_do_resize`** — called when `need_resize` is set. Calls `screen_resize` to re-query terminal dimensions, then re-initialises or clamps the scene to fit the new size. Resets `frame_time` and `sim_accum` so the dt measurement does not include resize latency.

---

## 9. Main Loop — Annotated Walk-through

```c
int main(void)
{
    /* ── setup ───────────────────────────────────────────────────────── */
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);   /* Ctrl-C  → running = 0        */
    signal(SIGTERM,  on_exit_signal);   /* kill    → running = 0        */
    signal(SIGWINCH, on_resize_signal); /* resize  → need_resize = 1    */

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns(); /* start of current frame         */
    int64_t sim_accum   = 0;          /* leftover ns from previous ticks */
    int64_t fps_accum   = 0;          /* ns accumulated for FPS counter  */
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── ① resize check ──────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset so dt doesn't include resize */
            sim_accum  = 0;
        }

        /* ── ② dt measurement ────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;  /* wall-clock ns since last frame */
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;  /* cap at 100ms */

        /* ── ③ sim accumulator (fixed-step physics) ──────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ render interpolation alpha ────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;
        /* alpha ∈ [0, 1): how far into the next (unfired) tick we are */

        /* ── ⑤ FPS counter (reads dt, no I/O) ───────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap BEFORE terminal I/O ────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
        /*
         * elapsed = time spent on physics + FPS counter this frame.
         * We sleep whatever is left of the 1/60s budget.
         * By sleeping HERE (before doupdate), the measurement excludes
         * terminal I/O latency, so the cap is stable regardless of
         * how long the terminal write takes.
         */

        /* ── ⑦ draw + present (one doupdate flush) ───────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ input (non-blocking) ──────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
```

**Why `elapsed = clock_ns() - frame_time + dt`:**

`frame_time` was set to `now` at step ②. `clock_ns() - frame_time` measures only what happened since then (physics, FPS counter). Adding `dt` back gives total time elapsed since the *previous* frame's end — the full frame budget used. Subtracting from `NS_PER_SEC/60` gives the sleep needed to hit exactly 60 fps.

---

## 10. Signal Handling and Cleanup

```c
static App g_app;   /* global so signal handlers can reach it */

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }
```

**`atexit(cleanup)`** — registers `endwin()` to run when the process exits for any reason. This restores the terminal (re-enable echo, show cursor, etc.) even if the program crashes or is killed.

**`SIGWINCH`** — sent by the terminal emulator whenever the window is resized. The handler sets `need_resize = 1`. The main loop checks this flag at the top of each iteration and calls `app_do_resize`. The flag pattern avoids calling ncurses functions from inside the signal handler (which is async-signal-unsafe).

**`SIGINT` / `SIGTERM`** — set `running = 0`. The main loop exits cleanly on the next iteration, calls `screen_free` → `endwin`, and returns normally.

**`(void)sig`** — suppresses the `-Wunused-parameter` warning. Signal handlers must have the signature `void f(int)` but the value is not needed here.

---

## 11. Adding a New Animation

Follow this checklist to add a new file to the project:

1. **Copy the §1–§8 structure** from `bounce_ball.c` or the closest existing file.

2. **§1 config** — define your physics constants, `SIM_FPS_DEFAULT`, `HUD_COLS`, `NS_PER_SEC`, `NS_PER_MS`, `TICK_NS`.

3. **§4 coords** — include only if your physics needs isotropic (square-pixel) coordinates. If your simulation works directly in cell space (like a grid CA), omit it.

4. **§5 physics struct** — define your simulation object. Store all state in pixel (or cell) coordinates. No ncurses types here.

5. **Tick function** — accept `float dt` (seconds). Apply forces, integrate, handle boundaries. No ncurses calls.

6. **`scene_draw`** — receive `float alpha`. Compute interpolated draw positions. Call `px_to_cell_x/y` once per object. Use `mvwaddch` / `wattron` / `wattroff` to write into `stdscr`.

7. **`screen_draw`** — call `erase()`, then `scene_draw(…, alpha)`, then write the HUD. Return without flushing.

8. **`screen_present`** — `wnoutrefresh(stdscr)` + `doupdate()`. One call per frame.

9. **Main loop** — follow the exact order: resize → dt → accumulator → alpha → FPS counter → **sleep** → draw → present → input. Do not move the sleep.

10. **Build line** — `gcc -std=c11 -O2 -Wall -Wextra yourfile.c -o yourname -lncurses -lm`

---

---

## 12. Software Rasterizer — raster/*.c

The `raster/` folder contains four self-contained software rasterizers that render 3-D geometry into the ncurses terminal using ASCII characters. They share an identical pipeline; only the mesh and (for `displace_raster.c`) the shader set differ.

Files:

| File | Primitive | Notes |
|---|---|---|
| `torus_raster.c`    | UV torus  | 4 shaders, always-on back-face cull |
| `cube_raster.c`     | Unit cube | 4 shaders, toggleable cull, zoom |
| `sphere_raster.c`   | UV sphere | 4 shaders, toggleable cull, zoom |
| `displace_raster.c` | UV sphere | 4 displacement modes, 4 shaders, zoom |

### Pipeline Overview

```
tessellate_*()          — build Vertex/Triangle arrays once at init
    ↓
scene_tick()            — rotate model matrix, recompute MVP each frame
    ↓
pipeline_draw_mesh()    — for every triangle:
    vert shader              VSIn → VSOut   (model → clip space)
    clip reject              all 3 verts behind near plane → skip
    perspective divide       clip → NDC → screen cell coords
    back-face cull           2-D signed area ≤ 0 → skip
    bounding box             clamp to [0,cols-1] × [0,rows-1]
    rasterize                for each cell in bbox:
        barycentric test         outside triangle → skip
        z-interpolate            z-test against zbuf → skip if farther
        interpolate VSOut        world_pos, world_nrm, u, v, custom[4]
        frag shader              FSIn → FSOut
        luma → dither → cbuf
    ↓
fb_blit()               — cbuf → stdscr → doupdate
```

### ShaderProgram and the Split Uniform Fix

All four raster files define:

```c
typedef struct {
    VertShaderFn  vert;
    FragShaderFn  frag;
    const void   *vert_uni;   /* passed to vert() */
    const void   *frag_uni;   /* passed to frag() */
} ShaderProgram;
```

**Why two uniform pointers instead of one:**

The vertex and fragment shaders can require *different* uniform struct types. In `displace_raster.c`, `vert_displace` needs `DisplaceUniforms` (contains `disp_fn`, `time`, `amplitude`, `frequency`) while `frag_toon` needs `ToonUniforms` (contains `bands`). With a single `void *uniforms` pointer, one of the two shaders would receive a pointer to the wrong struct. When it casts and dereferences it — for example, calling `du->disp_fn(...)` where `disp_fn` is at a byte offset that lies inside `ToonUniforms.bands` — the result is a null or garbage function pointer and an immediate segfault.

The fix is a separate pointer per shader stage. The pipeline passes each pointer only to the shader that owns it:

```c
/* pipeline_draw_mesh — vertex stage */
sh->vert(&in, &vo[vi], sh->vert_uni);

/* pipeline_draw_mesh — fragment stage */
sh->frag(&fsin, &fsout, sh->frag_uni);
```

`scene_build_shader` sets both pointers appropriately for each shader combination:

| Active shader | `vert_uni`     | `frag_uni`        |
|---|---|---|
| phong         | `&s->uni`      | `&s->uni`         |
| toon          | `&s->uni`      | `&s->toon_uni`    |
| normals       | `&s->uni`      | `&s->uni`         |
| wireframe     | `&s->uni`      | `&s->uni`         |

For the toon case, `vert_uni = &s->uni` (the vertex shader only needs `Uniforms`) and `frag_uni = &s->toon_uni` (`frag_toon` needs `ToonUniforms.bands`). This is safe because `ToonUniforms` leads with `Uniforms base` as its first member, so `(const Uniforms *)vert_uni` is a valid alias — zero-offset rule.

This fix was applied to all four raster files even though only `displace_raster.c` strictly requires it, to prevent the same class of crash if shaders are ever extended.

### Framebuffer

```c
typedef struct { float *zbuf; Cell *cbuf; int cols, rows; } Framebuffer;
typedef struct { char ch; int color_pair; bool bold; } Cell;
```

- `zbuf[cols*rows]` — float depth buffer, initialised to `FLT_MAX` each frame
- `cbuf[cols*rows]` — output cell buffer, written to `stdscr` by `fb_blit()`
- `luma_to_cell(luma, px, py)` — Bayer 4×4 ordered dither maps `[0,1]` luminance to a Paul Bourke ASCII density character and one of 7 ncurses color pairs

### Mesh and Tessellation

```c
typedef struct { Vec3 pos; Vec3 normal; float u,v; } Vertex;
typedef struct { int v[3]; } Triangle;
typedef struct { Vertex *verts; int nvert; Triangle *tris; int ntri; } Mesh;
```

Each `tessellate_*()` function allocates and fills `Vertex` and `Triangle` arrays once at startup. The pipeline indexes into `mesh->verts` using `tri->v[vi]` — all indices are guaranteed in-bounds by the tessellation loop construction.

### Vertex and Fragment Shaders

Shaders are plain C functions accessed through function pointers:

```c
typedef void (*VertShaderFn)(const VSIn *in,  VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in,  FSOut *out, const void *uni);
```

**Vertex shaders** — transform model-space position to clip space, output world-space position and normal for lighting:
- `vert_default` — standard MVP transform (torus/cube/sphere)
- `vert_normals` — same + packs world normal into `custom[0..2]`
- `vert_wire`    — same + pipeline injects barycentric coords into `custom[0..2]`
- `vert_displace` — displaces position along normal before transforming (displace only)

**Fragment shaders:**
- `frag_phong`   — Blinn-Phong + gamma correction
- `frag_toon`    — quantised diffuse (N bands) + hard specular threshold
- `frag_normals` — world normal → RGB (debug view)
- `frag_wire`    — `min(custom[0..2])` edge distance → discard interior, draw edge

**`custom[4]`** in `VSOut`/`FSIn` is a general-purpose interpolated payload. Each shader pair uses it differently:
- phong/toon — unused
- normals    — `custom[0..2]` = world normal components
- wireframe  — `custom[0..2]` = per-vertex barycentric identity vector `(1,0,0)/(0,1,0)/(0,0,1)`; after barycentric interpolation across the triangle, `min(custom[])` is the distance to the nearest edge

### Displacement (displace_raster.c)

Four displacement modes, each a pure function `float fn(Vec3 pos, float time, float amp, float freq)`:

| Mode   | Formula |
|---|---|
| RIPPLE | `sin(time + r*freq) * amp * taper`  — concentric rings from equator |
| WAVE   | `sin(time + x*f + y*f + z*f) * amp` — diagonal travelling wave |
| PULSE  | `breathe * amp * exp(-r*falloff)`   — whole sphere breathes |
| SPIKY  | `pow(|sin(x*f)*sin(y*f)*sin(z*f)|, 0.6) * amp` — spiky ball |

After displacing `pos += N * d`, the surface normal must be recomputed. The method is central difference:

```
d_t = displace(pos + eps*T) - displace(pos - eps*T)   ← finite diff along tangent T
d_b = displace(pos + eps*B) - displace(pos - eps*B)   ← finite diff along bitangent B
T'  = T*(2*eps) + N*d_t    ← displaced tangent vector
B'  = B*(2*eps) + N*d_b    ← displaced bitangent vector
N'  = normalize(cross(T', B'))
```

`DisplaceUniforms` extends `Uniforms` with `disp_fn`, `time`, `amplitude`, `frequency`, `mode`. It leads with `Uniforms base` so `&disp_uni` casts cleanly to `const Uniforms *` inside fragment shaders that only need the base lighting fields.

---

## 13. Cellular Automata & Particle Grid Sims — fire.c, smoke.c, aafire_port.c, sand.c

### fire.c — Three-Algorithm Fire

`fire.c` writes a `[rows × cols]` float heat grid and renders it through a shared Floyd-Steinberg + perceptual-LUT pipeline.  Three algorithms are runtime-switchable with `a`:

**Algo 0 — Doom CA** (Fabien Sanglard, 2013):
```
Bottom row: arch-shaped fuel (warmup-scaled at startup, wind-shifted each tick)
For each cell (x, y) from top to rows-2:
  rx = x + rand(-1, 0, +1)          lateral jitter → sideways flicker
  src = heat[y+1][rx]               sample ONE cell below (not average)
  heat[y][x] = max(0, src − decay)
```
Decay is computed from screen height: `avg_d = MAX_HEAT / (rows × CA_REACH_FRAC)`.  Split into `d_base = avg_d × 0.55` and `d_rand = avg_d × 0.90` so flame peak sits at 75% of terminal height on any terminal size.

**Algo 1 — Particle fire:**
`MAX_FIRE_PARTS = 800` particles are born at the arch source zone, rise with upward `vy`, carry a `heat` value that fades by `decay = 1/lifetime` each tick.  On each tick all active particles advance, then the heat grid is cleared and rebuilt by 3×3 Gaussian splat:
```
Kernel (sum=1): corners=0.0625, edge-midpoints=0.125, centre=0.25
splat3x3(heat, cx, cy, p->heat)
```
This fills the grid densely enough that the same Floyd-Steinberg pipeline renders it identically to the CA mode.

**Algo 2 — Plasma tongues:**
No persistent grid state.  Each tick the entire grid is recomputed from three overlapping sine harmonics:
```
tongue(x, t) = PLASMA_BASE
             + H1_AMP × sin(wx × H1_XFREQ + t × H1_TSPD)
             + H2_AMP × sin(wx × H2_XFREQ − t × H2_TSPD)
             + H3_AMP × sin(wx × H3_XFREQ + t × H3_TSPD)
heat(x, y) = clamp((ny − (1 − tongue)) / tongue, 0, 1)
```
where `ny = y/rows` (0 at top, 1 at bottom — same direction as CA).

**§5 Shared helpers** (called by all three algos):
- `warmup_scale(g)` — ramp 0→1 over WARMUP_TICKS=80; increments counter
- `advance_wind(g)` — wind_acc += wind, wraps at cols
- `arch_envelope(x, cols, wind_acc)` — squared arch weight [0,1] at column x
- `seed_fuel_row(g, wscale)` — write arch-shaped fuel to bottom row
- `splat3x3(heat, cols, rows, cx, cy, v)` — 3×3 Gaussian deposit

**All magic floats are named `#define` presets in §1**, grouped: source zone, CA decay, particle physics, plasma harmonics.  No unnamed float literals appear in any algo function.

**Rendering pipeline (shared):**
```
heat[][] → gamma pow(v, 1/2.2) → Floyd-Steinberg dither → perceptual LUT → mvaddch
```
Diff-based clearing: cold cells that were also cold last frame get no write.  Only cells hot last frame but cold now emit an explicit `' '`.

**Sections:** §1 presets · §2 clock · §3 theme · §4 grid · §5 shared helpers · §6 CA algo · §7 particle algo · §8 plasma algo · §9 scene · §10 screen · §11 app

---

### smoke.c — Three-Algorithm Smoke

`smoke.c` shares the same Floyd-Steinberg + perceptual-LUT rendering pipeline as fire.c, with a softer ramp `" .,:coO0#"` and six smoke-specific color themes (gray/soot/steam/toxic/ember/arcane).  Three algorithms switchable with `a`:

**Algo 0 — CA diffusion:**
Same structure as fire.c CA but with `CA_JITTER_RANGE = ±2` columns (vs fire's ±1) giving smoke's characteristic lateral billow.  Decay targets `CA_REACH_FRAC = 0.60` of rows (vs fire's 0.75) for a lower, denser column.

**Algo 1 — Particle puffs:**
`MAX_PARTS = 400` particles; density rendered = `life²` (quadratic fade for long soft trails vs fire's linear fade).  Bilinear splat (2×2 tent filter) rather than 3×3 Gaussian — smoke puffs are softer and larger than fire embers.

**Algo 2 — Vortex advection:**
Three point vortices orbit the screen centre.  Velocity at any point via 2D Biot-Savart:
```
vx += strength × (−dy) / (r² + VORT_EPS)
vy += strength × ( dx) / (r² + VORT_EPS)
```
Semi-Lagrangian advection: `new_d[p] = bilinear(old_d, p − v(p) × ADV_DT) × (1 − decay) + source(p)`.  `ADV_DT = 0.8` keeps advection stable.

**Key structural differences from fire.c:**
- `WARMUP_CAP = 200` (vs fire's implicit cap at WARMUP_TICKS) prevents counter overflow
- Vortex presets stored as `static const float VORT_ORB_FRACS[]`, `VORT_STRENGTHS[]` etc. at file scope — not local arrays in `vortex_init()`
- Wind accumulated **once** in `scene_tick()` before dispatch; `vortex_tick()` does not touch `wind_acc`

**Sections:** §1 presets · §2 clock · §3 theme · §4 shared helpers · §5 CA algo · §6 particle algo · §7 vortex algo · §8 scene · §9 screen · §10 app

---

### aafire 5-Neighbour CA

`aafire_port.c` samples five neighbours instead of three:

```c
heat[y][x] = (heat[y+1][x-1] + heat[y+1][x] + heat[y+1][x+1]
              + heat[y+2][x-1] + heat[y+2][x+1]) / 5 - minus[y]
```

The two extra terms (`y+2` row) make the flame rise slower and form rounder blobs. `minus[y]` is a precomputed per-row decay LUT — stronger decay near the top, weaker at the bottom — normalised to the terminal height so flame height is consistent at any terminal size.

### Falling Sand — Gravity CA

`sand.c` processes cells bottom-to-top (critical: top-to-bottom would let grains teleport multiple cells in one pass):

```
For each grain at (y, x):
  try fall straight down   → (y+1, x)
  if blocked, try diagonal → (y+1, x±1)  — direction randomised each grain
  if blocked, try wind drift → (y, x±1)
  otherwise: stationary
```

Fisher-Yates shuffle is applied to the column scan order each tick to remove the left/right scan bias that otherwise makes all sand pile to one side.

---

## 14. Flow Field — flowfield.c, complex_flowfield.c

Both files share the same fundamental architecture: a 2-D angle grid is computed each tick, particles sample it via bilinear interpolation, and trails are drawn with direction glyphs. They differ in the variety of field generators and the sophistication of the color system.

---

### flowfield.c — Single Perlin noise field

#### Architecture

```
noise_init()           — build 256-element permutation table
scene_init()           — allocate particle pool + angle grid
each tick:
  field_update(time)   — resample noise at each grid cell → angle[y][x]
  particle_tick(p)     — bilinear-sample angle at float position,
                         apply velocity, age, wrap/respawn
scene_draw(alpha)      — draw particles at interpolated position,
                         choose direction char from velocity angle
```

#### Perlin Noise to Flow Vector

Two noise samples at offset coordinates produce independent noise values; `atan2` converts them to a direction angle:

```c
float nx = perlin2d(x * freq + 0, y * freq + 0, time * 0.3f);
float ny = perlin2d(x * freq + 100, y * freq + 100, time * 0.3f);
angle[y][x] = atan2f(ny, nx);
```

3 octaves of fBm are summed (each doubling frequency, halving amplitude) before computing the angle, giving the field fine-grained detail without high-frequency noise at large scales.

#### Bilinear Field Sampling

Particles are at float positions; the angle grid is integer-indexed. Bilinear interpolation samples smoothly between grid cells:

```c
float angle = lerp(lerp(grid[y0][x0], grid[y0][x1], fx),
                   lerp(grid[y1][x0], grid[y1][x1], fx), fy);
```

Without this, particles would snap between discrete angle values every time they cross a grid cell boundary — visible as sudden direction changes.

#### Ring Buffer Trails

Each particle maintains a ring buffer of its last N positions. The draw function iterates the ring from newest to oldest, drawing older positions dimmer. The head index advances each tick overwriting the oldest entry — O(1) per tick, no shifting.

---

### complex_flowfield.c — Four field types, cosine palettes, colormap mode

#### Architecture

```
noise_init()         — 256-element permutation table (same as flowfield.c)
scene_init()         — allocate angle grid + particle pool + vortex state
color_apply_theme()  — pre-bake 16 cosine palette samples into ncurses pairs
each tick:
  field_tick()       — dispatch to active field type → angle[y*cols+x]
  particle_tick(p)   — bilinear-sample, move, update head_pair from angle
scene_draw()         — background mode → particle trails
```

#### Four Field Types (cycle with 'a')

| Type | Generator | Visual character |
|------|-----------|-----------------|
| 0 Curl noise | Central-difference curl of scalar fBm: `Vx=∂ψ/∂y`, `Vy=−∂ψ/∂x` | Divergence-free, no sources/sinks, infinite looping |
| 1 Vortex lattice | 6 Biot-Savart point vortices on an orbiting ring, alternating CCW/CW | Spinning whirlpool patterns |
| 2 Sine lattice | `Vx=sin(x·fx+t)+sin(y·fy−t·0.7)`, `Vy=cos(x·fx−t·0.5)+cos(y·fy+t·0.3)` | Standing-wave interference |
| 3 Radial spiral | Polar: tangential + pulsing radial `W·sin(t)·cos(θ)` | Breathing galaxy spiral |

#### Curl Noise — Divergence-Free Field

A scalar potential ψ(x,y,t) is built from layered Perlin noise. The curl of this scalar field gives a 2-D divergence-free vector field:

```c
float dn = noise_fbm(x,     y + eps, t, octaves);
float ds = noise_fbm(x,     y - eps, t, octaves);
float de = noise_fbm(x+eps, y,       t, octaves);
float dw = noise_fbm(x-eps, y,       t, octaves);
Vx =  (dn - ds) / (2 * eps);   /* +∂ψ/∂y */
Vy = -(de - dw) / (2 * eps);   /* −∂ψ/∂x */
```

Divergence-free means `∂Vx/∂x + ∂Vy/∂y = 0` everywhere — no cells act as sources or sinks. Particles orbit indefinitely without clustering or dispersing, producing the looping smoke-like motion that pure angle noise cannot achieve.

#### Vortex Lattice — Biot-Savart

N_VORT=6 vortices sit on a ring that rotates VORT_ORB_SPD rad/tick. At each cell, velocity is the superposition of all vortex contributions:

```c
for (int i = 0; i < N_VORT; i++) {
    float dx  = x - vort_cx[i];
    float dy  = (y - vort_cy[i]) / CELL_AR;   /* visual-pixel correction */
    float r2  = dx*dx + dy*dy + VORT_EPS;
    float str = (i % 2 == 0) ? +VORT_STRENGTH : -VORT_STRENGTH;
    vx += str * (-dy) / r2;
    vy += str * ( dx) / r2;
}
```

`CELL_AR=0.5` corrects `dy` to visual-pixel space so orbits appear circular on screen despite terminal characters being taller than wide. `VORT_EPS=5.0` softens the singularity over ~2-cell radius.

#### Cosine Palette — 16 Pre-Baked Color Pairs

Instead of hardcoding 8 xterm color indices per theme, `complex_flowfield.c` uses the cosine palette formula to generate 16 pairs at theme-change time:

```c
/* color(t) = a + b * cos(2π * (c*t + d)) */
for (int i = 0; i < 16; i++) {
    float t  = (float)i / 15.f;          /* evenly spaced t ∈ [0,1] */
    int   fg = cos_to_xterm256(theme, t); /* → 16 + 36r + 6g + b     */
    init_pair(CP_BASE + i, fg, COLOR_BLACK);
}
```

Particles map their movement angle to a pair index: `angle_to_pair(angle) = CP_BASE + (int)(normalized_angle * 16) % 16`. The palette is a smooth hue cycle so adjacent angle directions get visually complementary colors rather than arbitrary jumps.

#### Three Background Modes (cycle with 'v')

| Mode | What is drawn | Visual effect |
|------|--------------|---------------|
| 0 blank | Nothing | Dark canvas, trail geometry only |
| 1 arrows | Dim `>^<v/\` glyphs tinted by palette | Wind-map aesthetic |
| 2 colormap | Bold direction glyphs, every cell colored by angle→palette | Full-screen procedurally generated art |

In colormap mode the entire terminal is painted with the cosine-palette hues corresponding to the local flow angle. Switching field type or theme completely transforms the image. Particle trails drawn on top at `A_BOLD` remain clearly visible.

---

## 15. Flocking Simulation — flocking.c

### Algorithm Modes

Five algorithms are runtime-switchable via the `mode` enum:

| Mode | Rule |
|---|---|
| Classic Boids | Separation (repel close neighbors) + Alignment (match average heading) + Cohesion (steer toward center of mass) |
| Leader Chase | Each follower steers directly toward its flock leader with a proportional velocity term |
| Vicsek | Align heading to average heading of neighbors within radius + add Gaussian noise; emergent phase transition |
| Orbit Formation | Followers maintain a fixed radius ring around the leader; angular position advances each tick |
| Predator-Prey | Flock 0 (predator) chases nearest other-flock boid; flocks 1–2 flee from flock 0's leader |

All modes run with the same physics integration (semi-implicit Euler) and toroidal boundary.

### Toroidal Topology

Boids wrap around screen edges instead of bouncing. Distance and steering calculations use toroidal shortest-path:

```c
static float toroidal_delta(float a, float b, float max)
{
    float d = a - b;
    if (d >  max * 0.5f) d -= max;
    if (d < -max * 0.5f) d += max;
    return d;
}
```

This produces `|d| ≤ max/2` — the shortest path across the wrap boundary. All neighbor detection, cohesion, alignment, and proximity brightness use this function.

### Cosine Palette Color Cycling

Flock colors rotate smoothly over time by re-registering ncurses color pairs mid-animation with the cosine palette formula:

```
c(t) = 0.5 + 0.5 × cos(2π × (t/period + phase))
```

Three independent phase offsets give independent RGB channels that cycle through perceptually balanced hues. The result is remapped to the xterm-256 color cube (`16 + 36r + 6g + b`, r/g/b ∈ [0,5]).

---

## 16. Bonsai Tree — bonsai.c

### Branch Growth Algorithm

Each branch is a struct with position, direction `(dx, dy)`, remaining life, and type. One growth tick = one step per active branch:

```c
void branch_step(Branch *b, Scene *sc)
{
    /* Wander: perturb dx/dy by random amount each step */
    /* Branch: when life crosses threshold, spawn child branches */
    /* Type-specific rules:
       trunk   — mostly upward, wide wander
       dying   — stronger gravity (curves down)
       dead    — no new branches, straight
       leafing — short horizontal bursts */
    draw_char_at(b->y, b->x, branch_char(b->dx, b->dy), b->color);
}
```

Character selection uses the same slope-to-char mapping as spring_pendulum.c: `|`, `-`, `/`, `\` based on the ratio `|dx| / |dy|`.

### Leaf Scatter

After a branch dies (life reaches 0), `leaf_scatter()` places leaf characters in a random radius around the tip position. Leaf chars are chosen from a configurable set and drawn with `A_BOLD` for a lighter look.

### Message Box with ACS Characters

The message panel is drawn using ncurses' portable box-drawing characters (`ACS_ULCORNER`, `ACS_HLINE`, `ACS_VLINE`, `ACS_LLCORNER`, etc.) — always correct regardless of terminal encoding.

```c
attron(COLOR_PAIR(6) | A_BOLD);
mvaddch(by, bx, ACS_ULCORNER);
for (int i = 1; i < box_w-1; i++) mvaddch(by, bx+i, ACS_HLINE);
mvaddch(by, bx + box_w - 1, ACS_URCORNER);
/* ... sides and bottom ... */
attroff(COLOR_PAIR(6) | A_BOLD);
```

`use_default_colors()` + `-1` background makes branches appear over transparent terminal backgrounds.

---

## 17. Constellation — constellation.c

### Interpolation: `prev/cur` Lerp

Stars store both previous and current positions. The draw function lerps between them:

```c
draw_px = s->prev_px + (s->px - s->prev_px) * alpha;
draw_py = s->prev_py + (s->py - s->prev_py) * alpha;
```

This is true interpolation (not forward extrapolation) because star velocities are too small for extrapolation to be numerically identical to lerp.

### Connection Line Rendering

Connection lines are drawn with stippling and `A_BOLD` based on distance ratio:
- `< 0.50` → bold solid line (close, bright)
- `< 0.75` → normal solid line (medium)
- `< 1.00` → normal stipple-2 line (far, dotted)

A `bool cell_used[rows][cols]` VLA prevents multiple lines from overwriting each other in dense regions — the first line to visit a cell claims it.

---

## 18. Raymarchers and Donut — raymarcher/*.c

### donut.c — Parametric Torus (No Mesh, No SDF)

`donut.c` computes the torus directly from trigonometry each frame:

```
For (θ, φ) over (0, 2π):
  x = (R + r·cosφ)·cosθ
  y = r·sinφ
  z = (R + r·cosφ)·sinθ
  Apply two rotation matrices A, B (the tumble)
  Project to screen with perspective
  Compute N·L for luminance
  Sort by depth, paint character
```

No pipeline, no barycentric interpolation, no z-buffer — just parametric evaluation + depth sort. It is a direct port of the classic "donut.c" algorithm.

### wireframe.c — 3-D Projected Edges

`wireframe.c` draws a cube's 12 edges by projecting the 8 vertices to screen space and connecting them with Bresenham lines. Character at each Bresenham step is selected by slope (`/`, `\`, `-`, `|`). Arrow keys rotate the model matrix in real time.

### SDF Sphere-Marcher Pipeline

`raymarcher.c`, `raymarcher_cube.c`, and `raymarcher_primitives.c` are sphere-marching SDF renderers. There is no mesh, no triangle rasterisation, no tessellation — geometry is defined implicitly by a Signed Distance Function.

**SDF rendering loop (per screen cell):**

```
For each cell (cx, cy):
  ro = camera origin (world space)
  rd = normalize(view_matrix * (cx, cy, focal_length))
  t  = 0.0
  for MAX_STEPS iterations:
      p = ro + t * rd
      d = sdf(p)           ← closest distance to any surface
      if d < HIT_EPSILON → hit at p
      if t > MAX_DIST    → miss (background)
      t += d             ← safe to step this far without crossing a surface
  if hit:
      N = finite_diff_normal(p)
      luma = blinn_phong(N, rd, light_dir)
      luma = pow(luma, 1/2.2)    ← gamma correction
      ch = bourke_ramp[luma]
      color_pair = grey_ramp_pair[luma]
      mvaddch(cy, cx, ch) with pair
```

**Finite-difference normal** (used in cube and primitives):
```c
vec3 normal(vec3 p) {
    float eps = 0.001f;
    return normalize((vec3){
        sdf(p + (eps,0,0)) - sdf(p - (eps,0,0)),
        sdf(p + (0,eps,0)) - sdf(p - (0,eps,0)),
        sdf(p + (0,0,eps)) - sdf(p - (0,0,eps)),
    });
}
```
6 extra SDF calls per hit point — more expensive than analytic normals but works for any SDF without derivation.

**Shadow ray:** After computing the surface normal, a secondary march is launched from `p + N*eps` toward the light source. If it hits before reaching the light, the point is in shadow and diffuse is set to 0. This doubles the SDF call count for lit points but produces hard shadows with no additional geometry.

**Gamma correction:** `pow(luma, 1/2.2)` compensates for the non-linear luminance response of terminal color indices. Without it, the grey ramp appears weighted toward the bright end — shadows are too light.

**Output path:** Unlike the raster files, the raymarchers do NOT use an intermediate `cbuf[]`. Each cell is computed and written to ncurses directly inside the march loop. There is no depth buffer — each ray tests only one pixel and the result is written immediately.

### Primitive Composition (raymarcher_primitives.c)

`raymarcher_primitives.c` composites multiple SDF shapes using set operations:

| Operation | SDF formula | Result |
|---|---|---|
| Union | `min(sdf_a, sdf_b)` | Both shapes visible |
| Intersection | `max(sdf_a, sdf_b)` | Only overlapping volume |
| Subtraction | `max(-sdf_a, sdf_b)` | B with A carved out |
| Smooth union | `smin(sdf_a, sdf_b, k)` | Blended merge |

Each primitive returns both a distance and a material ID. The march tracks which material's ID is returned at the closest distance — this maps to a different color pair for each primitive in the scene.

```
Primitives in scene: sphere, box, torus, capsule, cone
Each has its own init_pair → different grey shade
```

The `smin` (smooth minimum) function blends two SDFs within radius `k`, producing organic-looking merged surfaces without any mesh boolean operations.

---

## 19. Particle State Machines — fireworks.c, brust.c, kaboom.c

### fireworks.c — Three-State Rocket

`fireworks.c` implements a two-level state machine: rockets and their particles each have independent states.

**Rocket states:**

```
IDLE ──(launch trigger)──→ RISING ──(apex reached)──→ EXPLODED ──(all particles dead)──→ IDLE
```

- `IDLE`: rocket is dormant, waiting for a timer or trigger
- `RISING`: rocket position integrates upward each tick with `vy -= gravity * dt`; drawn as `'|'` with `A_BOLD`
- `EXPLODED`: rocket spawns N particles in a radial burst; rocket itself is hidden; state persists while any particle has `life > 0`

**Particle lifecycle:**
```
spawn: position = rocket apex, velocity = random radial
each tick: px += vx*dt; py += vy*dt; vy += gravity*dt; vx *= drag; life -= decay
draw: A_BOLD when life > 0.6; A_DIM when life < 0.2; base otherwise
```

Each particle holds an independent color from the 7 spectral pairs — not inherited from the rocket.

### brust.c — Staggered Burst Waves

`brust.c` differs from `fireworks.c` in two architectural ways:

1. **Staggered waves** — particles are not all spawned at t=0. Each wave has a `spawn_delay` offset, creating a multi-ring explosion that expands over time rather than all at once.

2. **Persistent scorch** — a `scorch[]` array accumulates footprint cells from past bursts. Every frame the scorch array is iterated and drawn with `A_DIM` before drawing active particles. The scorch array is never cleared — it is an unbounded accumulation (capped by `MAX_SCORCH` at compile time).

```
Each explosion frame:
  1. Draw scorch[] with A_DIM (persistent from all past bursts)
  2. Draw flash cross (center * + 4 cardinals +) with A_BOLD — frame 0 only
  3. Draw active particles life-gated brightness
  4. Add to scorch[] any particles that just died
```

`ASPECT = 2.0f` is applied directly in cell-space x-coordinates when spawning particles — no pixel space, just multiply particle x-velocity by 2 to compensate for non-square cells.

### kaboom.c — Deterministic LCG Explosions

`kaboom.c` separates rendering into a pre-render phase and a blit phase:

```
blast_render_frame(blast, frame_idx) → Cell cbuf[rows*cols]
    for each active element in the blast shape:
        compute (cx, cy) from frame_idx + element properties
        cbuf[cy*cols + cx] = {ch, color_id}

blast_draw(cbuf, rows, cols)
    for each non-empty Cell in cbuf:
        attron(COLOR_PAIR(cell.color_id))
        mvaddch(cy, cx, cell.ch)
        attroff(...)
```

**LCG determinism:** The blast shape is generated from a seed via a Linear Congruential Generator. Same seed → identical explosion every invocation. This allows replaying the same explosion without storing its output.

**3-D blob z-depth coloring:** Blob elements are computed in 3-D space and projected. Z-depth selects both the character and the color:
- Far (z > 0.8×persp): `'.'` in `COL_BLOB_F` (faded)
- Mid: `'o'` in `COL_BLOB_M`
- Near (z < 0.2×persp): `'@'` in `COL_BLOB_N` (intense)

**6 blast themes** each define `flash_chars[]` and `wave_chars[]` strings. The wave character at each ring is selected by `wave_chars[ring_variant % len]` — different themes produce visually distinct blast shapes from the same geometry.

---

## 20. Matrix Rain — matrix_rain.c

### Two-Pass Rendering Architecture

`matrix_rain.c` splits each frame into two distinct rendering passes because no single pass can produce both the persistent fade texture and the smooth head motion simultaneously.

```
screen_draw():
  erase()
  ① pass_grid(alpha)      — draw persistent grid cells
  ② pass_heads(alpha)     — draw interpolated column heads
  HUD
  doupdate()
```

**Pass 1 — Grid texture:**

The `grid[rows][cols]` array is a simulation-level persistence texture. Each cell stores a shade value (0 = empty, 1–6 = FADE to BRIGHT). The simulation tick `grid_scatter_erase()` stochastically decrements shade values each tick. Pass 1 iterates all non-zero grid cells and draws them with `shade_attr(shade)`:

```c
for each (r, c) where grid[r][c] != 0:
    attron(shade_attr(grid[r][c]))
    mvaddch(r, c, glyph_for(grid[r][c]))
    attroff(...)
```

**Pass 2 — Interpolated column heads:**

Each active column has a float `head_y` that advances each tick by `speed`. For rendering, the head position is forward-extrapolated:

```c
draw_head_y = col->head_y + col->speed * alpha;
int draw_row = (int)floorf(draw_head_y + 0.5f);  /* round-half-up */
```

The head character is drawn at `draw_row` with `HEAD` shade (`A_BOLD`). Cells directly below the head get progressively dimmer shades in the same pass — this is what produces the bright-tip-with-fading-tail look.

### Theme System

`matrix_rain.c` supports 5 themes, hot-swapped at runtime by calling `theme_apply(idx)`. Each theme re-registers all 6 color pairs with new xterm-256 indices. The pair numbers used in draw code do not change — only the colors behind them change:

```c
void theme_apply(int t) {
    const Theme *th = &k_themes[t];
    init_pair(1, th->fade_col,   COLOR_BLACK);
    init_pair(2, th->dark_col,   COLOR_BLACK);
    init_pair(3, th->mid_col,    COLOR_BLACK);
    init_pair(4, th->bright_col, COLOR_BLACK);
    init_pair(5, th->hot_col,    COLOR_BLACK);
    init_pair(6, th->head_col,   COLOR_BLACK);
}
```

Theme swap takes effect immediately on the next `doupdate()` — there is no transition frame.

### Shade Gradient

The 6-level `Shade` enum maps directly to a `shade_attr()` function that returns a combined `attr_t`:

| Shade | Attribute | Visual |
|---|---|---|
| FADE | `A_DIM` | Barely visible tail residue |
| DARK | base | Dark mid trail |
| MID | base | Normal trail |
| BRIGHT | `A_BOLD` | Bright trail near head |
| HOT | `A_BOLD` | Very bright, close to head |
| HEAD | `A_BOLD` | Sharpest character, leading edge |

The same pair number can produce three brightness tiers (dim / base / bold) — tripling the apparent gradient resolution with no extra color pairs.

---

## 20b. Matrix Snowflake — matrix_snowflake.c

Two independent real-time simulations rendered on one screen, each drawn in its own layer.

### Two-Layer Architecture

```
Layer 1 (background): Matrix rain
  — g_streams[COLS_MAX]: one RainStream per column (head float, trail, speed)
  — g_rain_ch[ROWS_MAX][COLS_MAX]: persistent char buffer, randomised near heads
  — rain_draw() skips cells where crystal.cells[r][c] != 0

Layer 2 (foreground): DLA crystal
  — Crystal.cells[ROWS_MAX][COLS_MAX]: 0=empty, n=color-pair-ID
  — xtal_draw() two-pass: glow halo (Pass 1), frozen cells (Pass 2)
  — crystal always drawn after rain so it is always on top (last-write-wins)
```

### Proximity-Spawn DLA Optimisation

Classic DLA spawns walkers at screen edges. On a 200-column terminal, expected random-walk distance from edge to center ≈ O((cols/2)²) steps. At 60 walkers × 30fps this could take minutes before growth starts.

Fix: spawn walkers on a circle at radius `max_frozen_dist + 8` around the crystal center. Walker only needs to travel ~8 cells to reach the crystal, regardless of terminal size.

```c
float angle = rand_float * 2 * PI;
int nc = cx0 + roundf(cosf(angle) * spawn_r);
int nr = cy0 + roundf(sinf(angle) * spawn_r / ASPECT_R);
```

ASPECT_R divides the row component so the spawn ring is circular in Euclidean space (not elliptical in terminal space).

### Crystal Lifecycle State Machine

```
STATE_GROW: rain_tick + walkers_tick each frame
    │
    │  max_frozen_dist >= trigger_dist (88% of max_dist)
    ▼
STATE_FLASH: all frozen cells → bold white '*', 28 frames
    │
    │  flash_tick == 0
    ▼
scene_reset(): xtal_init + walkers_init   (rain never resets)
    │
    └──────────────────────────────► STATE_GROW
```

### Rain Char Buffer

`g_rain_ch[ROWS_MAX][COLS_MAX]` stores the current character at every terminal cell. Updated each tick:
- Near stream head (depth 0–2): 67% chance of new random char per cell → fast head flicker
- Ambient: 4% of all cells randomised → background shimmer in idle regions

This decouples char update rate from stream position and avoids re-computing chars at draw time.

### D6 Symmetry

Identical to snowflake.c — see section 22. The same `CA6[]`/`SA6[]` tables and `xtal_freeze_symmetric()` function are used. 12 positions frozen per walker stick event (6 rotations × 2 reflections) with ASPECT_R=2.0 coordinate correction.

---

## 21. Documentation Files Reference

| File | Purpose |
|---|---|
| `Architecture.md` | Framework design, loop mechanics, coordinate model, per-subsystem deep dives including fractal systems |
| `Claude.md` | Build commands for all files; brief per-file description |
| `Master.md` | Long-form essays on algorithms, physics, and visual techniques; section K covers fractal systems |
| `Visual.md` | ncurses field guide: V1–V9 covering every ncurses technique with What/Why/How explanations and code; includes per-file reference (V9) for all files, Quick-Reference Matrix, Technique Index |
| `COLOR.md` | Color technique reference covering every color trick — mechanism, code pattern, visual effect; includes escape-time, distance-based, and IFS vertex coloring |

---

## 22. Fractal / Random Growth — fractal_random/

All eight files in `fractal_random/` share the §1–§8 framework. They differ from the physics-based animations in one structural way: the §4 section is a **Grid** (2-D cell buffer) rather than a coordinate-space converter. The grid owns the simulation state; physics ticks write into it; screen_draw reads it.

```
Grid  ──  typed cell array (uint8_t cells[ROWS][COLS])
       ──  stores color-index per cell, 0 = empty
       ──  drawn by iterating all cells, looking up COLOR_PAIR(color)
```

### DLA — snowflake.c and coral.c

**Diffusion-Limited Aggregation (DLA)** grows a crystal by releasing random walkers. A walker moves one cell per tick in a random direction. When it lands adjacent to an already-frozen cell it freezes in place and becomes part of the aggregate.

The key loop:

```c
while (!frozen) {
    walker_step(&w);          /* one random move               */
    if (grid_adjacent(g, w.col, w.row)) {
        grid_freeze(g, w.col, w.row, color);
        frozen = true;
    }
    if (out_of_bounds(w))
        respawn(&w);          /* replace escaped walker        */
}
```

Parameters that shape the crystal:
- **N_WALKERS** — more walkers → denser, faster growth
- **Sticking probability** — < 1.0 gives smoother, more coral-like branches

### D6 Hexagonal Symmetry with Aspect Correction

`snowflake.c` freezes all 12 D6 images simultaneously. Given a new frozen cell `(col, row)` relative to the grid centre `(cx, cy)`, the 12 images are generated by applying all combinations of `R(k×60°)` and a reflection.

Because terminal cells are not square (cell height ≈ 2 × cell width), rotation must happen in Euclidean space:

```c
float dx = col - cx;
float dy = row - cy;
float dx_e = dx;
float dy_e = dy / ASPECT_R;    /* to Euclidean */

/* rotate 60° */
float rx_e =  dx_e * COS60 - dy_e * SIN60;
float ry_e =  dx_e * SIN60 + dy_e * COS60;

int new_col = cx + (int)roundf(rx_e);
int new_row = cy + (int)roundf(ry_e * ASPECT_R);  /* back to cell */
```

This ensures that the 6 rotated images are truly 60° apart in physical pixels rather than appearing squished.

### Anisotropic DLA — coral.c

`coral.c` biases DLA to grow upward like coral polyps:

- **8 seed cells** along the bottom row initialise the aggregate
- Walkers spawn at the **top** of the screen and drift downward with 50% probability
- **Sticking probability by direction**: below = 0.90, side = 0.40, above = 0.10

This breaks the circular symmetry of standard DLA: branches grow predominantly upward, side branches angle outward, and caps are rare. The visual result resembles real coral or branching trees rather than a circular snowflake.

Color is assigned by height: bottom rows get deep colors, upper tips get lighter ones. Six vivid color pairs cycle through the height bands.

### Chaos Game / IFS — sierpinski.c and fern.c

The **chaos game** (Iterated Function System) generates a fractal attractor by repeatedly applying a randomly chosen affine transform to a single point. After a warm-up period, the orbit of that point traces the attractor exactly.

For the Sierpinski triangle the three transforms are all "move halfway to vertex v":

```c
static const float vx[3] = { V1X, V2X, V3X };
static const float vy[3] = { V1Y, V2Y, V3Y };
int v = rand() % 3;
*x = (*x + vx[v]) * 0.5f;
*y = (*y + vy[v]) * 0.5f;
```

N_PER_TICK iterations per tick build the triangle gradually. Color tracks which vertex was chosen on the last step — each of the three main sub-triangles is always one hue.

### Barnsley Fern IFS Transforms

`fern.c` uses four transforms selected by weighted probability (cumulative):

| Name  | Prob | a     | b     | c     | d     | e    | f    |
|-------|------|-------|-------|-------|-------|------|------|
| stem  | 1%   | 0.00  | 0.00  | 0.00  | 0.16  | 0.00 | 0.00 |
| main  | 85%  | 0.85  | 0.04  | −0.04 | 0.85  | 0.00 | 1.60 |
| left  | 7%   | 0.20  | −0.26 | 0.23  | 0.22  | 0.00 | 1.60 |
| right | 7%   | −0.15 | 0.28  | 0.26  | 0.24  | 0.00 | 0.44 |

Transform: `x' = a*x + b*y + e`,  `y' = c*x + d*y + f`.

The fern's natural aspect ratio (y ∈ [0,10], x ∈ [−2.5,2.5]) gives a y/x ≈ 4 ratio. Combined with ASPECT_R=2, a naïve scale would produce a very narrow fern. The fix is independent x and y scales:

```c
g->scale_y = (float)(rows - 3) / (FERN_Y_MAX - FERN_Y_MIN);
g->scale_x = (float)(cols) * 0.45f / (FERN_X_MAX - FERN_X_MIN);
```

### Escape-time Sets — julia.c and mandelbrot.c

Both files compute the classic escape-time iteration `z → z² + c` with `MAX_ITER` maximum steps.

- **Julia**: `c` is a fixed complex constant; `z` starts at the pixel coordinate. Six presets cycle through different `c` values that produce distinct shapes.
- **Mandelbrot**: `z` starts at 0; `c` is the pixel coordinate. Six presets select different zoom regions.

Coloring uses a fractional escape ratio `frac = iter / MAX_ITER`:

```c
if (frac < THRESHOLD)    return COL_BG;      /* inside set   */
if (frac < 0.30f)        return COL_C2;      /* slowest band */
if (frac < 0.55f)        return COL_C3;
if (frac < 0.75f)        return COL_C4;
                         return COL_C5;      /* fastest band */
```

`julia.c` uses a fire palette (white → yellow → orange → red); `mandelbrot.c` uses an electric neon palette (magenta → purple → cyan → lime → yellow).

### Fisher-Yates Random Pixel Reveal

Rather than computing pixels row-by-row (which would look like a scan line), both julia.c and mandelbrot.c reveal pixels in a uniformly random order using a pre-shuffled index array:

```c
/* init: fill then shuffle */
for (int i = 0; i < n; i++) g->order[i] = i;
for (int i = n - 1; i > 0; i--) {
    int j = rand() % (i + 1);
    int tmp = g->order[i]; g->order[i] = g->order[j]; g->order[j] = tmp;
}

/* tick: process PIXELS_PER_TICK indices */
for (int k = 0; k < PIXELS_PER_TICK && g->pixel_idx < n; k++, g->pixel_idx++) {
    int idx = g->order[g->pixel_idx];
    int row = idx / g->cols, col = idx % g->cols;
    /* compute and plot */
}
```

This gives the impression of the image crystallising from scattered noise rather than being drawn line by line.

### Koch Snowflake — koch.c

The Koch snowflake is built by recursive midpoint subdivision. Given a segment from P to Q, the new midpoint M is:

```c
/* midpoint vector */
float dqpx = (qx - px) / 3.0f;
float dqpy = (qy - py) / 3.0f;
/* rotate +60° to get outward bump */
float mx = px + dqpx + COS60 * dqpx - SIN60 * dqpy;
float my = py + dqpy + SIN60 * dqpx + COS60 * dqpy;
```

Each level replaces every segment with four (P→A, A→M, M→B, B→Q), growing the segment count by 4× per level. The starting triangle uses clockwise winding and circumradius = 1. With R(+60°) and CW winding, M lies outside the triangle on each edge — producing outward bumps, not inward ones.

Segments are rasterized onto the terminal grid using Bresenham's line algorithm. An adaptive `segs_per_tick = n_segs / 60 + 1` ensures each level takes approximately 2 seconds to draw regardless of segment count.

### Recursive Tip Branching — lightning.c

`lightning.c` grows lightning as a fractal binary tree rather than DLA. Each active tip advances one cell downward per tick with a persistent lean bias (−1 = left, 0 = straight, +1 = right). After `MIN_FORK_STEPS` steps, a tip may fork into two children with lean biases `±1` from the parent.

State machine:

```
ST_GROWING  — tips advance and fork
ST_STRIKING — all tips reached the ground; full bolt briefly displayed
ST_FADING   — bolt fades out; then scene resets
```

Glow is a Manhattan-radius-2 halo drawn around every frozen cell:
- Distance 1 → inner corona (`|`, teal dim)
- Distance 2 → outer halo (`.`, deep blue dim)

Color by row depth: top third = light blue (xterm 45), middle = teal (51), bottom = white (231). Active tips are drawn as `!` bright white to show the wavefront.

### Buddhabrot Density Accumulator — buddhabrot.c

`buddhabrot.c` renders orbital trajectories of the Mandelbrot iteration as a 2-D density map. Unlike escape-time renderers that color each pixel by how quickly c escapes, the Buddhabrot accumulates visit counts: every orbit point that lands within the display region increments `counts[row][col]`.

**Two-pass sampling per tick:**

```
Pass 1 — escape test:  iterate z→z²+c, record whether |z|>2 within max_iter
Pass 2 — orbit trace:  if mode condition met, iterate again, ++counts[row][col]
```

This avoids storing orbit buffers. Pass 1 is cheap; Pass 2 only runs for qualifying samples.

**Cardioid/bulb rejection (Buddha mode only):**
```c
float q = (cr-0.25f)*(cr-0.25f) + ci*ci;
if (q*(q+cr-0.25f) < 0.25f*ci*ci) return;   /* main cardioid */
if ((cr+1.0f)*(cr+1.0f)+ci*ci < 0.0625f)    return;   /* period-2 bulb */
```

**Mode-aware log normalization:**
Anti-mode creates extreme dynamic range (attractor cells: millions of hits; transient cells: 1–5 hits). `t = log(1+count)/log(1+max_count)` compresses this. The invisible floor is mode-dependent:
- Buddha: floor=0.05 (preserve low-density orbital structure)
- Anti: floor=0.25 (suppress transient noise dots)

**Grid struct** holds a static `uint32_t counts[80][300]` buffer (~96 KB). `max_count` is tracked incrementally — updated whenever a cell's count exceeds the current maximum.

**Five presets:** buddha-500, buddha-2000, anti-100, anti-500, anti-1000. After TOTAL_SAMPLES=150000 the image holds briefly then advances to the next preset.

### Pascal-Triangle Formation Flying — bat.c

`bat.c` is an artistic demo with three groups of ASCII bats flying outward from the terminal centre. Each group uses a filled Pascal-triangle formation.

**Formation layout:**
Row r contains r+1 bats. Total bats for n_rows = (n_rows+1)*(n_rows+2)/2. Flat index k maps to row and position via triangular-number inverse:
```c
int r = 0;
while ((r+1)*(r+2)/2 <= k) r++;
int pos = k - r*(r+1)/2;
*along = -(float)r * LAG_PX;
*perp  = ((float)pos - (float)r * 0.5f) * SPREAD_PX;
```

**World-space rotation:** formation offsets (along/perp) are rotated into world space using the group's flight angle. Leader sits at apex (along=perp=0); row 1 has two bats at ±SPREAD_PX/2 perpendicular; row 2 has three at −SPREAD_PX, 0, +SPREAD_PX; and so on.

**Live resize:** `+`/`-` keys change n_rows (1–6) while groups fly. New bats are placed at `leader_px + along*cos(angle) - perp*sin(angle)` without interrupting motion.

**Wing animation:** four-frame cycle per group (`/`, `-`, `\`, `-`) advances each tick. All bats in a group share the same phase, giving a synchronized flap.

---

## 23. FDTD Wave Equation — fluid/wave.c

`wave.c` simulates the scalar 2-D wave equation on the terminal grid with five togglable oscillating sources.

**Three-buffer FDTD:**
Each step uses three planes: `u_prev`, `u_cur`, `u_new`. The explicit finite-difference scheme:
```
u_new[r][c] = 2·u_cur[r][c] − u_prev[r][c]
            + c² · (u[r+1][c]+u[r-1][c]+u[r][c+1]+u[r][c-1] − 4·u[r][c])
```
After computing `u_new`, the buffers rotate: `u_prev ← u_cur`, `u_cur ← u_new`. CFL stability requires `c·√2 ≤ 1`; the code uses `c = 0.45` for a comfortable margin.

**Signed amplitude → 9-level ramp:**
The wave amplitude is real-valued and signed. Negative values (troughs) map to cool/dim colours; near-zero maps to blank (background shows through); positive values (crests) map to warm/bright colours. This gives the interference fringes immediate visual depth.

**Five sources with offset frequencies:**
Each of the five point sources drives a sinusoidal oscillation at a slightly different frequency (e.g. 0.10, 0.11, 0.12 …). Beating between adjacent sources creates slowly shifting fringe patterns. Any source can be toggled on/off with keys `1`–`5`.

**Damping:**
A per-step damping factor `DAMP` (default ≈ 0.993) multiplies `u_cur` after each step. Without damping the grid would saturate; too much damping kills patterns immediately.

---

## 24. Gray-Scott Reaction-Diffusion — fluid/reaction_diffusion.c

`reaction_diffusion.c` implements the Gray-Scott model — two chemicals U and V that react and diffuse.

**The equations:**
```
dU/dt = Du·∇²U  −  U·V²  +  f·(1−U)
dV/dt = Dv·∇²V  +  U·V²  −  (f+k)·V
```
U is the substrate (starts at 1, replenished by feed rate `f`). V is the catalyst (starts near 0, removed by kill rate `k`). The nonlinear term `U·V²` drives pattern formation.

**9-point isotropic Laplacian:**
```
∇²u ≈ 0.20·(N+S+E+W) + 0.05·(NE+NW+SE+SW) − u
```
This is more rotationally symmetric than the 4-point stencil, producing rounder spots and stripes.

**Dual-grid ping-pong:**
Two grids `grid[0]` and `grid[1]` alternate as read and write each step. The active index `g_cur` flips between 0 and 1. This ensures simultaneous update — every cell reads the same generation.

**Parameter presets:**
Small shifts in (f, k) produce radically different patterns. The 7 presets span the known regime map:
| Preset | f | k | Visual |
|---|---|---|---|
| Mitosis | 0.0367 | 0.0649 | dividing spots |
| Coral | 0.0545 | 0.0620 | branching coral |
| Stripes | 0.0300 | 0.0570 | zebra stripes |
| Worms | 0.0780 | 0.0610 | writhing worms |
| Maze | 0.0290 | 0.0570 | labyrinth |
| Bubbles | 0.0980 | 0.0570 | expanding rings |
| Solitons | 0.0250 | 0.0500 | stable moving blobs |

**Warm-up:** 600 steps are pre-computed before the first frame so patterns are already developed at startup.

---

## 25. Chaotic Double Pendulum — physics/double_pendulum.c

`double_pendulum.c` integrates the exact Lagrangian equations of motion for a double pendulum (equal masses, equal arm lengths).

**Equations of motion:**
Let `δ = θ₁ − θ₂`, `D = 3 − cos 2δ` (always ≥ 2, never singular):
```
θ₁'' = [−3g sin θ₁ − g sin(θ₁−2θ₂) − 2 sin δ (ω₂²L + ω₁²L cos δ)] / (L·D)
θ₂'' = [2 sin δ (2ω₁²L + 2g cos θ₁ + ω₂²L cos δ)] / (L·D)
```

**RK4 integration:**
4th-order Runge-Kutta is used rather than Euler or Verlet. On chaotic trajectories, lower-order integrators accumulate phase errors on the Lyapunov time-scale (~3–5 s) that are visually indistinguishable from genuine chaos — RK4 keeps the simulation honest for longer.

**Ghost pendulum for chaos demonstration:**
A dim second pendulum starts with `θ₁ + GHOST_EPSILON` (a tiny offset). Both trajectories are identical at first; after ~3–5 s they diverge completely. The HUD shows the angular separation growing exponentially, making Lyapunov sensitivity tangible.

**Ring-buffer trail:**
The second bob traces a `TRAIL_LEN`-entry ring buffer of past positions. The most recent entries render in bright red/orange; older entries fade to dim grey. This reveals the complex attractor geometry.

---

## 26. Fourier Epicycles — artistic/epicycles.c

`epicycles.c` computes the DFT of a sampled parametric path and animates the rotating arm chain whose tip traces the original shape.

**Algorithm:**
1. Sample the chosen shape into `N_SAMPLES = 256` complex points `z[k]`.
2. Compute DFT: `Z[n] = Σ_k z[k]·exp(−2πi·n·k/N)`
3. Sort coefficients by amplitude `|Z[n]|/N` (largest arm first).
4. Animate: at angle `φ`, arm `n` contributes `(|Z[n]|/N) · exp(i·(freq_n·φ + arg Z[n]))`, chained from the pivot.
5. Auto-add one epicycle every `AUTO_ADD_FRAMES` to show convergence from a single arm to the full shape.

**Five shapes:** Heart, Star, Trefoil, Figure-8, Butterfly — sampled as parametric `(x(t), y(t))` curves over `[0, 2π]`.

**Subpixel coordinates:**
Same `CELL_W=8` subpixel scheme as in flocking.c — arm tip positions are stored in pixel space and divided by `CELL_W`/`CELL_H` at draw time so diagonal arms look proportional on the terminal.

**Orbit circles:**
The `N_CIRCLES` largest-amplitude arms draw their orbit circles as faint `·` dots. Toggled with `c`. Seeing the circles clarifies why certain shapes emerge from the harmonic chain.

---

## 26b. Fourier Art — artistic/fourier_art.c

`fourier_art.c` extends the epicycle concept to user-drawn paths. Instead of preset mathematical shapes, the user draws any curve on screen with cursor keys; the DFT of that path drives the epicycle animation.

**Two-state machine:**
```
STATE_DRAW  ─── ENTER/g ──► STATE_PLAY
              ◄── r ─────────
```

**DRAW mode:** Cursor `@` moves with arrow keys / WASD. Each movement records the new `(col, row)` position into a raw buffer (up to RAW_MAX=8192 points). Visited cells display as `.` dots. The HUD shows a live point count.

**Arc-length resampling (`resample_and_center`):**
Raw cursor positions are unevenly spaced — fast movement creates long gaps, slow movement clusters many points. To get a clean DFT, the path is resampled to exactly `N_SAMPLES=256` equally arc-length-spaced complex points:
1. Compute cumulative arc length along the raw path.
2. Walk `k = 0..255`, target `s_k = k/256 × total_arc`.
3. Binary-search for the raw segment containing `s_k`, linearly interpolate.
4. Subtract the centroid so the path is centered at the origin.

**Scale fitting:** After resampling and centering, `max_r` is the furthest point from the origin. `g_scale = 0.40 × min(screen_width, screen_height) / max_r` fits the reconstruction inside 40% of the screen.

**PLAY mode:** Identical to `epicycles.c` — sorted epicycles, auto-add convergence, Bresenham arm lines, orbit circles, colour-graded trail. The pivot is fixed at screen centre (the centroid of the drawn path maps here).

**Key difference from epicycles.c:**
- Source of samples: user input vs. parametric formula
- Arc-length resampling step (epicycles.c samples at uniform parameter t, which is fine for smooth shapes but wrong for a cursor path where t ≠ arc-length)
- No preset cycling; `r` returns to DRAW mode to draw a new path

*Files: `artistic/fourier_art.c`*

---

## 27. Leaf Fall — artistic/leaf_fall.c

`leaf_fall.c` draws a procedural ASCII tree, then rains its leaves down in matrix-rain style before resetting with a new tree.

**State machine:**
```
ST_DISPLAY (2.5 s) → ST_FALLING (until all leaves settle) → ST_RESET (0.7 s) → new tree
```
Each state has its own tick rate. `spc` skips the current state.

**Recursive tree generation:**
The trunk is drawn as a vertical stack of `|` characters. Branches are grown recursively from tip to base using a branching stack (`BSTACK`). Each branch has a heading angle, length, and depth. At each node the tree probabilistically splits into two sub-branches with slightly randomised spread angles. Leaves are placed at terminal branch tips.

**Matrix-rain leaf fall:**
Each leaf column has a `head` position (white `*` character) moving downward at `FALL_NS` rate and a trail of `TRAIL_LEN = 7` green characters fading behind it. Columns start with a staggered delay (`MAX_START_DELAY` ticks) so the fall looks organic rather than synchronized. Once the head reaches the bottom the column turns off.

**Algorithmic variation:**
Each new tree uses fresh random parameters for trunk height (45–65% of screen), spread angle, and branch-length ratio. No two trees look identical.

---

## 28. String Art — geometry/string_art.c

`string_art.c` recreates the mathematical string art technique: N nails on a circle, connected by threads using a multiplier rule.

**Thread mathematics:**
For multiplier `k`, thread `i` connects nail `i` to nail `round(i·k) mod N`. As `k` varies continuously, distinct shapes emerge at rational `k` values:
- `k = 2` → cardioid
- `k = 3` → nephroid
- `k = 4` → deltoid
- `k = 5` → astroid

**Speed modulation near integers:**
The drift speed of `k` is modulated so `k` slows dramatically as it approaches integer values, then accelerates through the irrational region. This ensures each named shape is visible long enough to appreciate:
```c
float dist = fabsf(g_k - roundf(g_k));
float mult = 0.15f + 1.70f * (dist * dist * 4.0f);
g_k += K_SPEED * mult;
```

**Slope-based thread characters:**
Each thread is drawn as a straight line using `mvaddch`. The character is chosen by the visual slope of the thread with aspect correction (`vs = |dy * 2 / dx|`):
- `vs < 0.577` → `-`
- `vs > 1.732` → `|`
- else sign-correct `/` or `\`

**Circle rim and nail markers:**
`RIM_STEPS = 2000` sample points draw the circle as `·` characters. Nails are drawn as bold `o` on top of the rim. This gives the user clear visual landmarks.

**Rainbow palette:**
12 fixed 256-colour indices cover the spectrum. Thread `i` uses `CP_T0 + (i % N_TCOLS)`, giving each nail its own color in the rainbow cycle.

---

## 29. 1-D Wolfram Cellular Automata — artistic/cellular_automata_1d.c

`cellular_automata_1d.c` animates all 256 Wolfram elementary rules as a build-down pattern that grows from a single cell at the top.

**Rule encoding:**
For a cell with left neighbour `l`, itself `m`, right neighbour `r`:
```c
next[c] = (rule >> ((l<<2) | (m<<1) | r)) & 1
```
The 8-bit rule number encodes the 8 possible 3-neighbor combinations.

**Build-down animation:**
Instead of scrolling, the CA pattern builds from row 0 downward. The current generation `g_gen` determines how many rows are visible. State machine:
- `ST_BUILD`: advance one row per `g_delay` ticks
- `ST_PAUSE` (`PAUSE_TICKS = 90`, ~3 s): hold the complete pattern before resetting to the next rule

**17 presets in 5 Wolfram classes:**
| Class | Color | Rules |
|---|---|---|
| Fixed | grey 244 | 0, 255 |
| Periodic | cyan 51 | 90, 18, 150, 60 |
| Chaotic | orange 202 | 30, 45, 22 |
| Complex | green 82 | 110, 54, 105, 106 |
| Fractal | yellow 226 | 90, 57, 73, 99, 126 |

**Title bar:**
Row 0 shows the rule name and number in `A_REVERSE` with the class color, immediately identifying the pattern type without obscuring the CA grid.

**Live digit input:**
Typing 1–3 digits accumulates a rule number. After 3 digits (or pressing Enter) the rule applies immediately, allowing any of the 256 rules to be explored without cycling.

---

## 30. Conway's Game of Life + Variants — artistic/life.c

`life.c` runs Conway's Game of Life and five rule variants on a toroidal grid with a population histogram.

**B/S bitmask rule encoding:**
Each rule is stored as two `uint16_t` values — `birth` and `survive` — where bit N is set if the count N triggers the transition:
```c
uint16_t bit = (uint16_t)(1u << n);   // n = neighbor count
uint8_t nxt  = cur ? ((survive & bit) ? 1 : 0)
                   : ((birth   & bit) ? 1 : 0);
```
This makes adding new rules trivial: just specify which neighbor counts trigger birth and survival.

**Six rules:**
| Rule | Birth | Survive | Character |
|---|---|---|---|
| Conway | 3 | 2,3 | classic |
| HighLife | 3,6 | 2,3 | glider self-replication |
| Day&Night | 3,6,7,8 | 3,4,6,7,8 | symmetric alive/dead |
| Seeds | 2 | — | explosive growth |
| Morley | 3,6,8 | 2,4,5 | chaotic structures |
| 2×2 | 3,6 | 1,2,5 | tiling block growth |

**Double-buffered grid:**
`g_grid[2][MAX_ROWS][MAX_COLS]` — `g_buf` indexes the active buffer, `1 - g_buf` is the next. After computing the next generation, `g_buf` flips. The old buffer becomes the write target for the following step.

**Gosper glider gun:**
The 36-cell pattern is hardcoded as `static const int GG[][2]` coordinates. Placed at offset `(g_ca_rows/4, 5)` to leave room for gliders to travel right.

**Population histogram:**
A `HIST_LEN = 512` entry ring buffer stores population counts. The bottom 3 rows of the screen render as a bar chart: bars grow upward, normalised to `[0, HIST_ROWS]`. This shows population oscillations and the transition to extinction.

---

## 31. Langton's Ant + Turmites — artistic/langton.c

`langton.c` simulates Langton's Ant and multi-colour turmites: automata where an ant walks a grid, changes cell states, and turns based on a rule string.

**Rule string encoding:**
The rule is a string of `R` and `L` characters. When the ant is on a cell in state `s`, it reads `rule[s % g_n_colors]` to determine turn direction. After turning, the cell advances to state `(s+1) % g_n_colors`. This generalises Langton's classic two-state rule to any number of colours.

**Ant step:**
```c
char turn  = g_rule[state % g_n_colors];
ant->dir = (turn == 'R') ? (ant->dir + 1) % 4   // clockwise
                         : (ant->dir + 3) % 4;   // counter-clockwise
g_grid[ant->r][ant->c] = (state + 1) % g_n_colors;
ant->r = (ant->r + DR[ant->dir] + g_rows) % g_rows;  // toroidal wrap
ant->c = (ant->c + DC[ant->dir] + g_cols) % g_cols;
```

**Multiple ants:**
1–3 ants share the same grid. They interact because they read and modify the same cell states. This produces emergent structures not seen with a single ant. Ants start spread across the grid at (0.5, 0.5), (0.35, 0.35), (0.65, 0.65) of the terminal dimensions.

**Speed:**
`STEPS_DEF = 200` ant steps per frame, doubling/halving with `+`/`-` up to `STEPS_MAX = 2000`. The classic "RL" rule requires ~10,000 steps before the highway emerges, so fast stepping is essential.

---

## 32. Chladni Figures — artistic/cymatics.c

`cymatics.c` animates Chladni nodal-line patterns — the standing-wave figures seen when sand collects on a vibrating plate.

**Formula:**
For mode `(m, n)` at normalized coordinates `(x, y) ∈ [0,1]`:
```
Z(x,y) = cos(m·π·x)·cos(n·π·y) − cos(n·π·x)·cos(m·π·y)
```
Nodal lines are where `Z ≈ 0`; antinodes are where `|Z|` is large.

**20 mode pairs:**
All `(m, n)` pairs with `1 ≤ m < n ≤ 7` — 20 distinct patterns from the simple (1,2) cross to the complex (6,7) lattice.

**Morphing animation:**
Instead of hard-cutting between modes, the `Z` value blends linearly:
```c
z = (1.0f - g_t) * z1 + g_t * z2   /* g_t advances 0 → 1 */
```
`MORPH_SPEED = 0.025f` per tick (~1.3 s morph). After morphing completes, the state returns to `ST_HOLD` for `HOLD_TICKS = 120` (~4 s) before the next morph.

**Nodal glow rendering:**
Rather than binary nodal/antinode rendering, five distance bands around the nodal line use progressively fainter characters:
```
|Z| < 0.04  →  '@'  (bright, CP_NODE white)
|Z| < 0.10  →  '#'  (bold)
|Z| < 0.18  →  '*'
|Z| < 0.28  →  '+'
|Z| < 0.40  →  '.'
```
Cells beyond 0.40 are blank. The `+` and `−` antinode regions are colored differently (CP_POS vs CP_NEG), while the nodal band is always white.

**Four themes:**
Classic (cyan/red), Ocean (blue/teal), Ember (orange/dark-red), Neon (green/magenta). Theme changes re-register color pairs live.

---

---

## 33. Wa-Tor Predator-Prey — artistic/wator.c

`wator.c` simulates Alexander Dewdney's 1984 Wa-Tor ocean: fish and sharks on a toroidal grid with local breed/starve rules producing global population oscillations.

**Grid state:** Four `uint8_t` arrays per cell — `g_type` (EMPTY/FISH/SHARK), `g_breed` (age counter), `g_hunger` (shark starvation counter), `g_moved` (double-process guard).

**Processing order:** All cell indices are shuffled with Fisher-Yates before processing each tick. Without shuffling, creatures always move right/down first, creating directional bias patterns. The shuffle makes movement isotropic.

**`g_moved` guard:** When a shark moves to (nr,nc) and eats the fish there, `g_moved[nr][nc] = 1`. Later in the same tick, when the shuffled order reaches that cell index, the shark's entry is skipped — preventing it from acting twice in one frame.

**Fish step:** `age++` → find random empty neighbour → move (set `g_moved[dest] = 1`) → if old enough, leave newborn at origin (not destination, so newborn doesn't act same tick).

**Shark step:** `breed++; hunger++` → die if `hunger > SHARK_STARVE` → else find fish neighbour (eat, reset hunger, set `g_moved`) → else find empty neighbour (move) → if `breed >= SHARK_BREED`, spawn child at origin.

**Dual histogram:** 4 bottom rows split into 2-row fish panel (cyan, scale = max_pop/2) and 2-row shark panel (red, scale = max_pop/10). Ring buffer stores last `g_cols` population samples for scrolling history. Shark scale compressed 5× because shark counts are far lower.

---

## 34. Abelian Sandpile — fractal_random/sandpile.c

`sandpile.c` drops grains one at a time onto the centre of a grid. Any cell with ≥ 4 grains topples: loses 4 grains, gives 1 to each cardinal neighbour. Cascades propagate via BFS until the grid is stable.

**BFS avalanche queue:**
```c
static int     g_qr[QMAX], g_qc[QMAX];
static uint8_t g_inq[MAX_ROWS][MAX_COLS];
static int     g_qhead, g_qtail;
```
`g_inq[]` deduplication ensures each cell enters the queue at most once per avalanche pass. `QMAX = MAX_ROWS × MAX_COLS + 1` (circular queue).

**`avalanche_step(full)` modes:**
- `full=1`: drain the queue to completion → entire cascade runs in one call (instant mode).
- `full=0`: process one cell then `break` → one topple per video frame (vis mode, toggle with `v`).

**wave_end bug:** The original code set `wave_end = g_qtail` at entry and looped only until `g_qhead == wave_end`. This missed cells enqueued *during* toppling — the cascade died after one ring. Fixed by looping on `g_qhead != g_qtail` with no snapshot.

**Coloring:** `GRAIN_CH[4] = {' ', '.', '+', '#'}` — blank, dim-blue, green, bright-gold for 0–3 grains. Unstable cells (≥4, visible only in vis mode) drawn as bright-red `*`. Drop point marked `@` when empty.

**Self-organised criticality:** After millions of grains the long-run stable pattern develops intricate fractal/quasi-crystalline 4-fold symmetry. Avalanche sizes follow a power-law distribution — most are tiny, occasionally one crosses the entire pile.

---

## 35. SDF Metaballs — raymarcher/metaballs.c

`metaballs.c` raymarchess a scene of 6 smooth-min-blended spheres on Lissajous orbits, with Phong shading, soft shadows, and curvature coloring.

**Smooth-min (polynomial, C¹):**
```c
float smin(float a, float b, float k) {
    float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
    return fminf(a, b) - h * h * k * 0.25f;
}
```
`k` is the blend radius. `k → 0`: hard min (separate spheres). Large `k`: wide neck (merged blob). The `h²·k/4` term widens the isosurface beyond either individual sphere.

**Tetrahedron normal (4 SDF evaluations vs 6):**
4 vertices `(±e,±e,±e)` sample the SDF and are combined to extract the gradient. Each output component uses `+f0 -f1 -f2 +f3` (or permutations) — exactly 4 evaluations for a full 3D gradient.

**Curvature coloring:** Laplacian of the SDF via 7-point stencil (6 neighbours + centre). For a sphere of radius r, Laplacian = 2/r ≈ 3.6 at the surface → maps to high band (warm color). Flat merged saddles → low band (cool color). 4 themes × 8 curvature bands = 32 color pairs initialised at startup.

**Soft shadows:** March from surface toward light; track `min(sk·h/t)` along the ray. Close-approach ratio gives penumbra shading. `SHADOW_K = 8.0f` controls hardness.

**Canvas downsampling (CELL_W=2, CELL_H=2):** Each canvas pixel covers 2×2 terminal cells. Reduces pixel count 4× while maintaining visual quality. Aspect correction: `phys_aspect = (ch·CELL_H·CELL_ASPECT) / (cw·CELL_W)` applied to vertical ray component.

**Lissajous orbits:** Ball i at position `(RX·sin(A[i]·t+PX[i]), RY·sin(B[i]·t+PY[i]), RZ·cos(C[i]·t))`. Phase offsets `PX[i] = i·2π/6` spread balls evenly at t=0. Different A/B/C frequencies per ball prevent synchronization.

---

## 36. Harmonograph / Lissajous — geometry/lissajous.c

`lissajous.c` draws an exponentially decaying dual-oscillator parametric curve (harmonograph) that morphs through figure-8s, trefoils, stars, and spirals as phase drifts.

**Parametric equations:**
```
x(t) = sin(fx·t + phase) · exp(−decay·t)
y(t) = sin(fy·t)         · exp(−decay·t)
```
`T_MAX = N_LOOPS·2π/min(fx,fy)` — always N_LOOPS=4 full cycles of the slower oscillator regardless of ratio. `DECAY = DECAY_TOTAL/T_MAX` — amplitude reaches ~1% at T_MAX for every ratio.

**Age-based rendering:** Iterates `i = N_CURVE_PTS-1 → 0` (oldest inner loops first, newest outer loops last). `age = i/(N-1)`: 0 = newest (full amplitude, outermost, brightest '#'), 1 = oldest (tiny amplitude, innermost, dimmest '.'). Newest overwrites shared cells, so the outer bright ring always wins.

**Phase dwell mechanism:** Symmetric Lissajous figures occur at phase multiples of `π/max(fx,fy)`. Drift is multiplied by `DWELL_SPEED=0.25` near each key phase, linearly ramping back to 1× within `DWELL_WIDTH=0.25` of the period:
```c
float kp   = (float)M_PI / fmaxf(fx, fy);
float pm   = fmodf(phase_x, kp);
float d    = fminf(pm, kp - pm);
float frac = d / (kp * DWELL_WIDTH);
float mul  = DWELL_SPEED + (1.0f - DWELL_SPEED) * fminf(frac, 1.0f);
```
This lets each named shape (Figure-8, Trefoil, Star…) dwell visibly before transitioning.

**Auto-ratio cycling:** After phase_x accumulates 2π, the ratio index advances and phase resets to 0. Eight ratios cycle in order: 1:2, 2:3, 3:4, 1:3, 3:5, 2:5, 4:5, 1:4.

**4 themes × 4 brightness levels = 16 color pairs + 1 HUD pair:**
- Level 0 = outermost (age≈0) → brightest character '#' + A_BOLD
- Level 3 = innermost (age≈1) → dimmest character '.'
- Themes: Golden (226,220,136,94), Ice (51,38,23,17), Ember (231,208,196,88), Neon (118,82,28,22)

---

---

## 37. Physics Simulations — physics/

### Lorenz Strange Attractor — lorenz.c

Integrates the three Lorenz ODEs with RK4 and projects the 3-D trajectory onto a 2-D terminal view that auto-rotates.

**ODEs:** `ẋ = σ(y−x)`, `ẏ = x(ρ−z)−y`, `ż = xy−βz` with σ=10, ρ=28, β=8/3. These canonical parameters produce the famous butterfly attractor.

**RK4 integration:** Each tick advances the state vector (x,y,z) by `DT_PHYS` using four slope evaluations — essential because the Lorenz system is stiff and Euler diverges in the chaotic region.

**3-D → 2-D projection:** The attractor is rotated by a slowly advancing `view_angle` around the Z axis: `px = x·cos(a) − y·sin(a)`, `py = x·sin(a) + y·cos(a)`. No perspective; orthographic projection is sufficient because the attractor has no depth ordering issues.

**Ring-buffer trail:** 1500-slot circular buffer stores past (x,y) pixel positions. Drawn from oldest (dim) to newest (bright) so the age gradient shows direction of travel. Color: red (newest) → orange → yellow → grey (oldest).

**Ghost trajectory:** A second integrator starts with a tiny ε offset. Lyapunov divergence makes the two paths separate exponentially — visible on screen as a magenta ghost that starts co-located then drifts away, demonstrating chaos sensitivity.

*Files: `physics/lorenz.c`*

---

### N-Body Gravity — nbody.c

20 point masses under mutual softened gravity, Verlet integrated, with trails showing orbital history.

**Softened gravity:** `F = G·m_i·m_j / (r² + ε²)` prevents singularities when two bodies pass close. ε=4 px keeps forces bounded while still allowing tight orbits and slingshot events.

**Verlet integration (O(n²) force loop):**
```c
for each body i:
    ax = ay = 0
    for each body j ≠ i:
        dx = x[j]-x[i], dy = y[j]-y[i]
        r2 = dx*dx + dy*dy + EPS*EPS
        f  = G * m[j] / (r2 * sqrtf(r2))
        ax += f*dx; ay += f*dy
    vx[i] += ax*dt; vy[i] += ay*dt
    x[i]  += vx[i]*dt; y[i] += vy[i]*dt
```

**Trail ring buffer:** 200 positions per body. Color fades: body color → dim → very dim. The trail shows orbital curvature, slingshot hyperbola, figure-8 etc.

**Optional central black hole:** A fixed massive body (50× normal mass) at screen centre creates a stable attractor with satellite orbits.

*Files: `physics/nbody.c`*

---

### Spring-Mass Cloth — cloth.c

`CLOTH_W × CLOTH_H` grid of masses connected by structural, shear, and bend springs. Top row pinned; gravity and wind perturb the free nodes.

**Physics: explicit spring forces + symplectic Euler.**
The original Verlet + Jakobsen (position-based constraints) approach was abandoned because sequential constraint projection propagates pin corrections through the entire cloth in one pass, eliminating all deformation regardless of force magnitude.

Force-based integration:
```c
/* 1. Spring force for each spring (a,b): */
dx = x[b]-x[a]; dy = y[b]-y[a];
dist = sqrtf(dx*dx + dy*dy);
stretch = dist - rest;
nx = dx/dist; ny = dy/dist;
vrel = (vx[b]-vx[a])*nx + (vy[b]-vy[a])*ny;
F = k*stretch + kd*vrel;   /* Hooke + damping */
ax[a] += F*nx/mass; ay[a] += F*ny/mass;
ax[b] -= F*nx/mass; ay[b] -= F*ny/mass;

/* 2. Symplectic Euler: */
vx[i] += ax[i]*dt; vy[i] += ay[i]*dt;
vx[i] *= DAMP; vy[i] *= DAMP;   /* global velocity damping */
x[i]  += vx[i]*dt; y[i] += vy[i]*dt;
```

**Spring constants:** K_STRUCT=400, K_SHEAR=100, K_BEND=40 (stiffness); KD_STRUCT=4, KD_SHEAR=2, KD_BEND=0.5 (damping). DAMP=0.9993 per tick.

**Render interpolation:** Each node stores a render snapshot `(rx,ry)` taken after the previous tick. Drawing lerps `draw = rx + (x−rx)*alpha` for smooth sub-tick motion.

**Modes:** `hanging` (top-row pinned, gravity down), `flag` (left-column pinned, wind rightward), `hammock` (two corner pins, gravity down).

*Files: `physics/cloth.c`*

---

### Magnetic Field Lines — magnetic_field.c

Visualises the static 2D magnetic field of 1–4 bar magnets (dipoles). Each dipole is modelled as a pair of equal-and-opposite magnetic monopoles. The field at any point is the Coulomb superposition:

```
B(r) = Σ_i  q_i · (r − r_i) / |r − r_i|³
```

with SOFT=0.8 cell-unit softening to avoid the singularity at each pole centre.

**Streamline tracing:** N_SEEDS=16 field lines are seeded on a small circle around every North pole. Each line is traced by 4th-order RK4 integration along the normalised B direction. At each step the local field vector is sampled four times (standard RK4 k1–k4) and averaged. The integrated direction is then mapped to a line character (`-`, `|`, `/`, `\`) or an arrow (`>`, `v`, `<`, `^`) every ARR_STRIDE=18 steps to show field orientation.

**Aspect correction:** Terminal cells are ~2× taller than wide. All y-components in `field_at()` are scaled by ASPECT_R=2.0 before computing distance, then divided back when returning B_y. This makes circular field patterns look circular rather than elliptical.

**Incremental reveal:** All lines are traced at scene initialisation. `lines_traced` starts at 0 and increments by `lines_per_tick` each tick, so lines appear to draw themselves progressively.

**Presets:**
| # | Name | Description |
|---|---|---|
| 0 | Dipole | Single bar magnet — closed loops from N to S |
| 1 | Quadrupole | Two orthogonal dipoles — X-type saddle null point at centre |
| 2 | Attract | Two magnets with opposite poles facing — dense convergent lines between them |
| 3 | Repel | Two magnets with same poles facing — divergent lines, null point between |

**Color by proximity:** Lines are coloured bright (CP_LINE_BRT) near the seed pole, fading to dim (CP_LINE_DIM) farther away — simulates field strength falloff.

*Files: `physics/magnetic_field.c`*

---

### Hanging Chain — chain.c

`chain.c` simulates a chain of point masses using **Position-Based Dynamics (PBD)** — a fundamentally different approach from the explicit spring forces in `cloth.c`.

**Algorithm per sub-step:**
```
1. Verlet predict (free nodes only):
     vx = (x - ox) * DAMP
     new_x = x + vx + wind * dt²
     new_y = y + vy + GRAVITY * dt²
     ox = x;  x = new_x          (velocity is implicit in (x - ox))

2. Constraint projection (N_ITER iterations):
     for each link (a, b):
       d  = b.pos − a.pos
       dist = |d|
       corr_ratio = (dist − rest) / dist
       if both free:  a += 0.5 * corr_ratio * d
                      b -= 0.5 * corr_ratio * d
       if a pinned:   b -= corr_ratio * d
       if b pinned:   a += corr_ratio * d
```

**Why PBD instead of springs:** Spring forces require a stiffness constant `k`; for a stiff chain, `k` must be large, which drives explicit Euler unstable (requires tiny dt). PBD enforces constraints geometrically — it moves nodes to satisfy the length constraint directly. The result is unconditionally stable at any stiffness; more iterations → stiffer chain, not instability.

**Tension coloring:** Each link is coloured by `stretch = |dist − rest| / rest`:
- Cyan (CP_LINK_LO): relaxed (stretch < 4%)
- Yellow (CP_LINK_MID): mildly stretched (4–12%)
- Red (CP_LINK_HI): very stretched (> 12%)

During fast swings, the outermost links carry the most centripetal load and turn red; the middle links stay cyan. This makes the tension distribution visually legible.

**Presets:**
| # | Name | Physics | Visual signature |
|---|---|---|---|
| 0 | Hanging | Top pinned, wind oscillates | Chain sways left-right, catenoid shape |
| 1 | Pendulum | Top pinned, 60° initial angle | Swings with wave patterns in the tail |
| 2 | Bridge | Both ends pinned, catenary sag | Bounces under vertical wind gusts |
| 3 | Wave | Top node driven sinusoidally | Traveling waves reflect at free bottom; standing wave modes emerge |

**Wave physics (preset 3):** The top node's position is set to `anchor_x + A·sin(ω·t)` before each sub-step. Wave speed in a hanging chain varies with depth: `c(y) ∝ √(T(y)/μ)` where tension `T(y)` is highest at the top. This dispersion means high-frequency components travel faster — the chain acts as a dispersive medium, and at certain driver frequencies a standing wave pattern locks in.

*Files: `physics/chain.c`*

---

### Slime Mold — fluid/slime_mold.c

`slime_mold.c` implements the **Jeff Jones (2010) Physarum polycephalum** agent model. Individual agents sense their chemical trail environment, turn toward the strongest signal, move, and deposit more trail. The trail diffuses and decays on a grid. From these simple local rules, the colony self-organises into a network topology that empirically approximates minimum Steiner trees — the same structure real slime molds use to connect food sources.

**Agent update (per tick, per agent):**
```
sense at (x,y) + SENSOR_DIST × (angle ± SENSOR_ANGLE) — 3 positions
  FL = trail_sample(forward-left)
  F  = trail_sample(forward)
  FR = trail_sample(forward-right)
if F ≥ FL and F ≥ FR: keep angle
elif FL > FR:          angle -= ROTATE_ANGLE
elif FR > FL:          angle += ROTATE_ANGLE
else:                  angle ±= ROTATE_ANGLE (random)
x += cos(angle) × STEP_SIZE
y += sin(angle) × STEP_SIZE
trail_deposit(x, y, DEPOSIT × food_bonus_if_near_food)
```

**Trail grid update (double-buffered):**
```
for each cell (r, c):
    avg = (3×3 neighbourhood sum) / 9
    buf[r][c] = (trail[r][c] × (1−DIFFUSE_W) + avg × DIFFUSE_W) × (1−DECAY)
memcpy(trail, buf)
```

Double buffering is critical: without a separate write buffer, cells at the top of the grid get averaged using already-updated neighbour values while cells at the bottom still use stale values, breaking the diffusion symmetry.

**Food sources:** Three food positions per preset deposit a FOOD_MIN_TRAIL floor and give agents a FOOD_BONUS=6× deposit multiplier when within FOOD_RADIUS pixels. This biases the network toward connecting food sources while letting the colony self-route the paths between them.

**Sensor constants (Jones 2010 defaults):**
- `SENSOR_ANGLE = π/4` (45°) — wide enough to discriminate left/right clearly
- `SENSOR_DIST = 4.0` — samples ahead of the agent's current cell
- `ROTATE_ANGLE = π/4` — one 45° step per tick; smaller values → gradual arcs
- `STEP_SIZE = 1.0` — one grid cell per tick

**Visualization:** Trail intensity mapped to 5 characters: `' '` → `'.'` → `'+'` → `'x'` → `'#'` → `'@'`. Food source cells display `'*'`. Five themes (Physarum, Cyan, Neon, Forest, Lava) each define 5 colour pairs for the intensity levels.

**Why emergent networks?** Each agent follows its own trail and the trails of its neighbours. High-traffic paths reinforce themselves (positive feedback). Low-traffic paths decay away (negative feedback). The result is a sparse, efficient network — the same mathematical structure that graph theorists spend NP-hard compute time trying to find.

*Files: `fluid/slime_mold.c`*

---

## 38. Artistic Effects — artistic/ & geometry/

### Aurora Borealis — aurora.c

Per-cell sinusoidal curtains with a vertical envelope, coloured by row position. No physics — purely mathematical.

**Per-cell formula:**
```
x = col/cols × 2π
y = row / (rows × AURORA_FRAC)       // 0=top, 1=bottom of aurora band

primary = sin(x·1.5 + t·0.20) × cos(y·3 + t·0.50 + x·0.25)
shimmer = cos(x·2.3 − t·0.15 + 1.0) × sin(y·5 + t·0.80 + x·0.40 + 2.5)
v       = primary·0.60 + shimmer·0.40    // combined, ∈ [−1,1]
v       = v·0.5 + 0.5                    // normalise to [0,1]
env     = sin(π·y)                        // 0 at top/bottom, 1 at midpoint
intensity = v × env
```

**Color zones by row fraction:** `y < 0.25` → magenta (fringe), `y < 0.55` → cyan (core), `y ≥ 0.55` → green (base). A_BOLD for high intensity, A_DIM for low.

**Characters by intensity:** `.` < 0.20, `:` < 0.40, `|` < 0.65, `!` < 0.82, `|` (bold) above.

**Deterministic star hash:** `h = col·1234597 ^ row·987659; h ^= h>>13; h *= 0x5bd1e995; h ^= h>>15`. If `(h & 0xFF) < STAR_THRESH` the cell is a star. Char chosen from `h>>8`, color from `h>>10`. Zero storage, no flicker.

*Files: `artistic/aurora.c`*

---

### Animated Voronoi — voronoi.c

24 seed points drift with Langevin Brownian motion. Each terminal cell is colored by its nearest seed (brute-force O(N) per cell per frame).

**Langevin motion (self-limiting drift):**
```c
vx += (-DAMP*vx + NOISE*randf()) * dt
```
`DAMP=2.0` s⁻¹ limits terminal speed. `NOISE=60` px/s² provides the random kick. Seeds bounce elastically off screen edges.

**Per-cell nearest-seed:** For each cell compute pixel-space Euclidean distance to all 24 seeds. Track `d1` (nearest) and `d2` (second nearest):
- `d1 < SEED_PX=12` → draw `O` A_BOLD (seed marker)
- `d2 - d1 < BORDER_PX=15` → draw `+` (cell boundary)
- else → draw `.` A_DIM (cell interior)

**Color:** Each seed has a fixed color pair (cycling through 6 pairs mod 6). Interior cells use the seed's color. Boundary cells use the average or seed color at A_DIM.

*Files: `geometry/voronoi.c`*

---

### Spirograph — spirograph.c

Three simultaneous hypotrochoid curves with slowly drifting parameters, rendered onto a float canvas that decays each tick.

**Hypotrochoid parametric equations:**
```
x = (R−r)·cos(t) + d·cos((R−r)/r · t)
y = (R−r)·sin(t) − d·sin((R−r)/r · t)
```
`R` = outer radius, `r` = inner radius, `d` = pen offset from inner circle centre.

**Parameter drift:** `r` oscillates as `r_base + r_amp·sin(drift)`. `drift` advances at `DRIFT_RATE·dt` per tick, slowly morphing the curve shape between different petal/star counts.

**Float canvas:** `canvas[MAX_ROWS][MAX_COLS]` stores per-cell intensity as float. Each tick: `canvas[row][col] *= FADE=0.985` (exponential decay). New curve points add 1.0. Intensity mapped to 5 brightness levels (`' '` `.` `:` `|` `@`).

**Three curves:** different `(R,r,d)` triples with phase offsets; cyan, magenta, yellow color pairs. Advancing `t` by `DELTA_T=0.08` per tick, `TRACE_STEPS=60` points traced per curve per tick for dense fill.

*Files: `geometry/spirograph.c`*

---

### Plasma — plasma.c

Classic demoscene plasma: sum of four sinusoids evaluated per terminal cell, mapped through a cycling 256-color palette.

**Plasma value per cell:**
```c
v = sin(col*f1 + t*s1)
  + sin(row*f2 + t*s2)
  + sin((col+row)*f3 + t*s3)
  + sin(dist*f4 + t*s4)   // dist = distance from screen centre
```
Normalized to [0,1] then shifted by `fmod(time*CYCLE_HZ, 1.0)` for palette cycling.

**4 frequency presets** (`FreqPreset` struct with f1–f4 spatial, s1–s4 speed):
- `gentle` — slow large waves
- `energetic` — faster medium waves
- `grand` — very slow large-scale waves
- `turbulent` — fast overlapping waves

**4 color themes** — each has 14 `PalEntry {pair, attr, ch}` entries. Characters: ` ` `.` `:` `+` `*` `#` etc. progress with intensity. Themes: rainbow, fire, ice, neon.

**Keys:** `space` pause, `p` cycle palette, `f` cycle frequency preset, `]`/`[` sim Hz.

*Files: `artistic/plasma.c`*

---

## 39. Penrose Tiling — fractal_random/penrose.c

Computes a P3 Penrose rhombus tiling per terminal cell using de Bruijn's pentagrid duality. No tile storage — O(1) per cell.

**De Bruijn pentagrid method:**
Five families of parallel lines. Family j has direction `e_j = (cos(2πj/5), sin(2πj/5))`. For a point (wx,wy) in pentagrid unit space:
```c
k[j] = floor(wx * COS5[j] + wy * SIN5[j])
S    = k[0] + k[1] + k[2] + k[3] + k[4]
```
Parity of S determines tile type: `S even → thick rhombus (72°)`, `S odd → thin rhombus (36°)`. Adjacent tiles always differ in S parity (crossing any grid line changes S by ±1), so thick and thin tiles only ever border each other.

**Aspect ratio correction:** Terminal cells are `CELL_H/CELL_W ≈ 2× taller` than wide. Cell (col,row) is mapped to pixel offset `(col·CELL_W, row·CELL_H)` before projecting to the pentagrid, keeping rhombus proportions correct.

**Animation:** The pixel frame rotates at `ROTATE_SPEED=0.04` rad/s. The tiling has 5-fold symmetry (period = 2π/5 ≈ 72°) but is globally aperiodic — no configuration ever repeats.

**Tile edge detection:** The fractional part of each projection `frac(k[j] proj)` measures distance to the nearest grid line. If `min_dist < BORDER=0.15`, the cell is on a tile edge and a directional line character is drawn:
```c
ang = 2π·j/5 + π/2 − view_angle   // grid line angle on screen
// Fold to [0,π) then pick: '-' '/' '|' '\'
```

**Interior fill:** `*` A_BOLD in warm gold/amber (thick), `.` in cool cyan/blue (thin). A hash `abs(k[0]·3+k[1]·7+k[2]·11+k[3]·13+k[4]·17) % 3` selects 1 of 3 color shades per type.

**Color pairs 8–12:** penrose-specific — gold (220), amber (214), aqua (87), lavender (147), pale yellow (228).

**Scale:** `SCALE_PX=80` → each rhombus side is 10 terminal columns wide so tile shapes are clearly legible.

*Files: `fractal_random/penrose.c`*

---

## 40. Diamond-Square Terrain — fractal_random/terrain.c

Generates a fractal heightmap via diamond-square midpoint displacement, erodes it with thermal weathering, then renders ASCII elevation contours via bilinear interpolation.

**Diamond-square algorithm** on a `(2^N+1) × (2^N+1)` grid (N=6, GRID=65):

*Diamond step:* For each square of side `step`, set the centre to the average of the four corners plus a random displacement `±scale`. *Square step:* For each diamond, set the edge midpoint to the average of the four diamond points plus `±scale`. After each full iteration: `scale *= ROUGHNESS=0.60`.

After generation: normalize to [0,1] by subtracting min and dividing by range.

**Thermal weathering erosion** (2 passes per tick, 4 neighbours):
```c
diff = h[y][x] - h[ny][nx]
if (diff > TALUS=0.022):
    move = EROSION_RATE=0.0012 * (diff - TALUS)
    h[y][x]   -= move
    h[ny][nx] += move
```
Material flows downhill when slope exceeds the `TALUS` angle. Mountains slowly crumble into plains over time.

**Bilinear interpolation:** The 65×65 grid is mapped to any terminal size. For cell (col,row), compute fractional grid position `(gx,gy)`, read the four surrounding grid heights, and interpolate: `h = h00·(1−fx)(1−fy) + h10·fx(1−fy) + h01·(1−fx)·fy + h11·fx·fy`. This eliminates blocky cell-size quantization artifacts.

**7 contour levels:**
| Height | Char | Color |
|---|---|---|
| < 0.15 | `~` | blue dim |
| < 0.28 | `~` | blue |
| < 0.42 | `.` | yellow dim |
| < 0.57 | `-` | green |
| < 0.72 | `^` | green bold |
| < 0.88 | `#` | orange |
| ≥ 0.88 | `*` | cyan bold |

*Files: `fractal_random/terrain.c`*

---

---

## 41. Forest Fire CA — misc/forest_fire.c

Implements the **Drossel-Schwabl (1992)** 3-state probabilistic cellular automaton modelling forest fire ecology. Each cell is EMPTY, TREE, or FIRE; all cells update simultaneously based on two parameters: `p` (growth probability) and `f` (lightning probability).

**Update rules (applied synchronously via double-buffer):**
```
FIRE  → EMPTY                              (burns out in one tick)
TREE  → FIRE    if any 4-neighbour is FIRE (fire spreads)
TREE  → FIRE    with probability f          (lightning)
TREE  → TREE    otherwise
EMPTY → TREE    with probability p          (regrowth)
EMPTY → EMPTY   otherwise
```

**Self-organised criticality:** At the critical ratio `p/f`, the distribution of fire sizes follows a power law `P(s) ∝ s^{−τ}` with `τ ≈ 1.19`. The system self-tunes to the edge of criticality — large fires are rare but possible at any scale, without any tuning of parameters. This is the same mechanism behind earthquake size distributions, solar flares, and species extinction events.

**Why the ratio matters, not the individual values:**
- Large `p/f` → trees grow faster than lightning ignites them → large dense clusters accumulate → catastrophic fires consume everything at once
- Small `p/f` → fires arrive before clusters grow large → small frequent fires, sparse landscape
- Critical `p/f` → cluster size distribution spans all scales simultaneously

**Double-buffer update:** Like Game of Life, `g_grid` stores the current state and `g_next` receives all computed next states before being memcpy'd back. Without this, a FIRE cell that burns out on the left side of the grid could no longer spread rightward — the state change would be visible mid-sweep.

**Flickering fire visualization:** FIRE cells alternate between `'*'` (CP_FIRE2, bright) and `','` (CP_FIRE1, dim) based on `(row + col + tick) & 1`. This checkerboard parity combined with frame advance creates a natural flicker without any random call per cell at draw time.

**Ash trail:** When FIRE → EMPTY, `g_ash[r][c] = 1` is set. The draw pass renders ash cells as `'.'` in CP_ASH (dark grey) for one tick. This gives the forest a burned-scar memory that fades after one frame, making fire propagation direction visually legible.

**Preset 3 (Smoulder):** Enables 8-neighbour fire spread (diagonal cells included). This doubles the possible spread directions, producing rounder fire fronts and slower smouldering expansion compared to the sharp diamond-shaped fronts of 4-neighbour spread.

*Files: `misc/forest_fire.c`*

---

---

## 42. Lattice Gas — fluid/lattice_gas.c

`lattice_gas.c` implements the **FHP-I model** (Frisch, Hasslacher & Pomeau, *PRL* 1986), the first cellular automaton proven to reproduce the Navier-Stokes equations at large scales. Particles live on a hexagonal lattice, one per direction per cell, encoded as bits 0–5 of a `uint8_t`.

**Hex direction encoding:**
```
bit 0 = E,  bit 1 = NE,  bit 2 = NW
bit 3 = W,  bit 4 = SW,  bit 5 = SE
Opposite of direction d: (d+3) % 6
```

**Two-phase update (collision then streaming):**

*Collision* — apply a 64-entry lookup table `g_col[parity][state]` that conserves particle count and momentum. Only five 6-bit patterns have non-trivial collisions:
```
Head-on 2-particle (ambiguous — resolved by (row+col)&1 parity):
  0x09 (E+W)   → 0x12 (NE+SW) [even] | 0x24 (NW+SE) [odd]
  0x12 (NE+SW) → 0x24         [even] | 0x09          [odd]
  0x24 (NW+SE) → 0x09         [even] | 0x12           [odd]
3-particle symmetric (unique rotation, no ambiguity):
  0x15 (E+NW+SW) → 0x2A (NE+W+SE)
  0x2A (NE+W+SE) → 0x15 (E+NW+SW)
All 59 remaining patterns: identity
```

*Streaming* — each particle moves to the hex neighbour in its direction (double-buffered into `g_buf`). Particles hitting a wall bounce back: direction reverses to `(d+3)%6`, particle stays at source cell. Grid edges wrap toroidally.

**Why the spatial parity alternation resolves head-on collisions:**
The three head-on pairs all share the same ambiguity (rotate CW or CCW?). Choosing based on `(r+c)&1` alternates between CW and CCW on a checkerboard pattern. This prevents the simulation from developing spurious global chirality — if all cells always chose CW, the fluid would systematically rotate, breaking the isotropy required for Navier-Stokes.

**Inlet condition:** After each collision+stream cycle, before the next one, the left column is seeded: each non-wall cell in column 0 has its East bit forced to 1 with probability `g_inlet_prob`. This drives a left-to-right flow. The right column wraps toroidally (particles exiting right re-enter left), creating a recirculating channel.

**Visualization:**
```
Character encodes density (popcount):  ' '(0) '.'(1) ':'(2) '-'(3) '+'(4) '#'(5) '@'(6)
Colour encodes horizontal momentum ×2:  E=+2, NE=+1, NW=-1, W=-2, SW=-1, SE=+1
  mx2 ≤ -3: CP_MW2 (cold)    strong westward
  mx2 < 0:  CP_MW1            weak westward
  mx2 = 0:  CP_MID            neutral
  mx2 > 0:  CP_ME1/ME2 (warm) eastward
```

**Aspect ratio correction for obstacles:** Terminal cells are ~2× taller than wide. The cylinder preset checks `dx² + (dy×ASPECT_R)² < R²` with `ASPECT_R=2.0`, so the ellipse in cell coordinates appears circular on screen.

**Why Navier-Stokes emerges:** At the coarse-grained level, the particle density and momentum obey continuity and momentum-conservation equations. The collision rules are designed so that the momentum flux tensor is isotropic (the hexagonal lattice has the minimum symmetry required), which gives the correct viscous stress tensor. The kinematic viscosity is ν ≈ 1/(6d(1−d)) where d is the mean occupation per direction — controlled by the inlet density.

*Files: `fluid/lattice_gas.c`*

---

## 43. Galaxy — artistic/galaxy.c

`galaxy.c` simulates a **spiral galaxy** using 3000 stars in exact circular orbits under a flat rotation curve. The spiral structure emerges entirely from the initial placement of stars on logarithmic spiral arms combined with differential rotation.

### Flat Rotation Curve

Real spiral galaxies have nearly constant tangential speed at all radii (the "rotation curve problem"). We encode this directly:

```
v_circ = V0 = const  →  ω(r) = V0 / r
```

Because ω is larger at small r, inner stars complete orbits faster than outer ones. This **differential rotation** slowly winds the arms up — exactly the phenomenon real astrophysicists study.

### Logarithmic Spiral Initialization

Each arm k has stars placed at:
```
θ = (k × 2π/N_ARMS) + WINDING × ln(r / r_min) + scatter
```
This is the equation of a logarithmic spiral: `r = r_min × exp((θ − θ_start) / WINDING)`. Stars start coherent on the arm; differential rotation deforms them over hundreds of ticks.

### Brightness Accumulator Grid

Instead of rendering individual star positions, stars are accumulated into a floating-point grid `g_bright[ROWS][COLS]`:

```c
g_bright[sy][sx] += 1.0f / g_steps;   /* weight normalised per frame */
```

Once per rendered frame, the grid is multiplied by `DECAY = 0.82`:
```c
g_bright[r][c] *= DECAY;              /* exponential fade trail */
```

The **steady-state** brightness for a cell receiving f stars/frame is:
```
B_ss = f / (1 − DECAY) = f × 5.56
```
Dense regions (bulge: many stars) saturate high; sparse regions (halo) stay dim. Both the character and colour are picked from the normalised brightness `b / b_max`.

### Galaxy Zones (Colour)

Screen distance from centre determines the colour zone, regardless of which stars currently occupy the cell:

```c
float rn = sqrt(dx²/rx² + dy²/ry²);
cp = (rn < 0.10) ? CP_CORE : (rn < 0.65) ? CP_DISK : CP_HALO;
```

Aspect-ratio correction (`ry = rx * 0.5`) makes the galaxy appear circular on terminals where cells are ~2× taller than wide.

### Population Breakdown

| Group | Count | Initialisation |
|---|---|---|
| Bulge | 600 | `r ~ |Gaussian(0, 0.07)|`, θ random |
| Arms  | 2100 | log-spiral arms, scatter ±ARM_SCATTER |
| Halo  | 300  | r ~ Uniform(0.35, 1.05), θ random |

### Keys

`a/A` cycle 2→3→4 arms (resets); `+/-` orbit speed; `t/T` theme; `r` reset; `p` pause.

*Files: `artistic/galaxy.c`*

---

## 44. Barnes-Hut — physics/barnes_hut.c

O(N log N) Barnes-Hut gravity with up to 800 bodies. A quadtree is rebuilt every physics tick from a static pool; direct body glyphs overlay a glow accumulator for clear visual feedback.

### Quadtree Build and Force Traversal

The tree is rebuilt from scratch each tick — `g_pool_top = 0` resets the 16,000-node static pool in O(1). Insertion updates the centre-of-mass incrementally on every node from root to leaf. Force traversal applies `s/d < θ=0.5`: if the cell is small and far, treat it as a single point mass; otherwise recurse into four children.

```c
/* Incremental COM update on insert */
float nm = n->total_mass + b->mass;
n->cx = (n->cx * n->total_mass + b->px * b->mass) / nm;
n->total_mass = nm;

/* Force criterion */
float s = n->x1 - n->x0;
if (s/d < THETA || is_leaf(n))
    apply_gravity_from_node(bi, n);   /* treat as point mass */
else
    for (c in 0..3) qt_force(child[c], bi, fx, fy);
```

### Galaxy Preset — Central BH + Keplerian Disk

Body 0 is a central black hole (`mass = N×3`, `anchor=true` — skipped in integration). Bodies 1..N−1 are placed at random radii and given Keplerian velocities `v = sqrt(G·M_bh/r)`. Differential rotation (inner bodies orbit faster) naturally shears the disk into spiral patterns over a few orbital periods.

### Dual-Layer Rendering

**Layer 1 — Glow grid:** Each body increments `g_bright[row][col]`. Grid decays by 0.93 per render frame. Normalised by `b_max` → character ramp `.,+oO`.

**Layer 2 — Direct body glyphs:** Every active body is drawn at its current cell with a character and color determined by `spd/g_v_max`: dim `,` (still) → bright `*` (fast). Drawn on top of the glow. Bodies are always visible regardless of speed.

### Quadtree Overlay

`'o'` key toggles ACS_HLINE/ACS_VLINE grid lines at depth ≤ 3 (≤ 64 cells). Below depth 3 the lines would obscure bodies; the `(cx1-cx0) < 2` guard prevents drawing lines inside sub-cell nodes.

### Keys

`q` quit, `p`/`space` pause, `r` reset, `1/2/3` preset, `t/T` theme, `o` overlay, `f` 4× fast-forward, `+/-` bodies, `g/G` gravity.

**Build:** `gcc -std=c11 -O2 -Wall -Wextra physics/barnes_hut.c -o barnes_hut -lncurses -lm`

*Files: `physics/barnes_hut.c`*

---

## 45. Analytic Wave Interference — fluid/wave_interference.c

Superposition of N sinusoidal point sources computed analytically — no FDTD grid, no time stepping of fields. Each source contributes `A·cos(k·r − ωt)`.

### Phase Precomputation

The spatial phase `k·r` is constant for a fixed source position. It is precomputed once into `g_phase[s][row][col]` when sources are placed or moved:

```c
float dx = (float)(c * CELL_W) - src.px;
float dy = (float)(r * CELL_H) - src.py;
g_phase[s][r][c] = K * sqrtf(dx*dx + dy*dy);
```

Per frame, each cell evaluates only `Σ cos(g_phase[s][r][c] − ω·t)` — one subtraction and one `cosf` per source per cell. The result in `[-1,+1]` maps to the signed 8-level ramp CP_N3..CP_P3.

### Presets

| Preset | Sources | Pattern |
|--------|---------|---------|
| Double slit | 2 in-phase point sources | Young's double-slit fringes |
| Ring | 6 evenly-spaced circular | Star interference |
| Linear | 4 evenly-spaced horizontal | Directional beam |
| Standing wave | 2 opposing phase sources | Nodes and antinodes |

*Files: `fluid/wave_interference.c`*

---

## 46. 7-Segment LED Morph — artistic/led_number_morph.c

168 particles (7 segments × 24 particles each) animate between digit shapes using spring physics. Each particle permanently belongs to one segment and springs toward that segment's on/off target.

### Segment Bitmask

```c
/* k_seg_mask[d] = bitmask of which 7 segments are active for digit d */
static const uint8_t k_seg_mask[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,   /* 0–4 */
    0x6D, 0x7D, 0x07, 0x7F, 0x6F,   /* 5–9 */
};
```

On digit change, each segment checks its bit; `on=true` parks particles at the segment's displayed positions, `on=false` parks them at a collapsed centre point.

### Spring Physics

Each particle: `F = SPRING_K·(target−pos) − DAMP·vel`. With SPRING_K=9, DAMP=5.5, the damping ratio ζ = DAMP/(2·sqrt(SPRING_K)) ≈ 0.92 — slightly underdamped, producing a small overshoot visible as a bounce at each digit transition.

### Orientation-Aware Characters

Horizontal segments use `─` (or `-`), vertical segments use `│` (or `|`). The segment orientation is stored in `k_seg_orient[]`; the draw function selects the character at render time, never at physics time.

*Files: `artistic/led_number_morph.c`*

---

## 47. Particle Number Morph — artistic/particle_number_morph.c

Up to 500 particles morph between bitmap font digits using greedy nearest-neighbour assignment and smoothstep LERP.

### Bitmap Font Expansion

A 9-row × 7-column binary font defines each digit. Each `#` pixel is expanded to a sub-grid of `ppr × ppc` particles (up to 3×4), scaled to terminal size. All 10 digit target grids are precomputed once at startup into `g_targets[10][N_PARTS]`.

### Greedy Nearest-Neighbour Assignment

On digit change, each of the new digit's `n_targets` positions is assigned the closest unassigned particle. O(n_targets × N_PARTS) ≈ 234,000 comparisons — runs in < 1ms. Unassigned particles are marked inactive and lerp to the centre (emerge/vanish effect).

### Smoothstep LERP + Origin Snapshot

```c
float st = t*t*(3.0f - 2.0f*t);          /* smoothstep */
p.x = p.ox + st * (p.tx - p.ox);         /* lerp from snapshot */
```

`ox/oy` is snapshotted at `digit_assign()` time — wherever the particle currently is (possibly mid-previous-morph). This means rapid digit changes produce smooth continuous motion rather than teleporting to the last digit's final position.

*Files: `artistic/particle_number_morph.c`*

---

## 48. Julia Explorer — fractal_random/julia_explorer.c

Split-screen interactive Julia/Mandelbrot explorer. Left panel: precomputed Mandelbrot map (computed once at startup, reused every frame). Right panel: Julia set for the current complex parameter c (recomputed every frame).

### Precomputed Mandelbrot Buffer

`g_mbuf[ROWS][COLS_M]` stores escape iteration counts for the Mandelbrot view at startup. The left panel renders from this buffer every frame at zero recomputation cost. Resize triggers a single recompute.

### Live Julia Panel

The right panel recomputes the Julia set for the current c each frame. The Julia iteration `z → z² + c` is the same as Mandelbrot but with fixed c and varying z₀ (pixel coordinates). Dynamic panel widths `g_mw / g_jw` rebalance on resize so both panels always show correctly aspect-corrected views.

### Auto-Wander

When auto-wander is active, c traces an ellipse near the Mandelbrot boundary:
```c
c.re = WANDER_R * cosf(wander_angle);
c.im = WANDER_R * WANDER_YSHRK * sinf(wander_angle);
```
`WANDER_R=0.72` places the ellipse in the bulge-arm junction — the most visually rich part of the Mandelbrot boundary.

*Files: `fractal_random/julia_explorer.c`*

---

## 49. IFS Chaos Game — fractal_random/barnsley.c

Five IFS presets (Barnsley Fern, Sierpinski Triangle, Lévy C Curve, Dragon Curve, Fractal Tree) rendered via the chaos game with log-density accumulation.

### Cumulative Probability Transform Selection

Each preset defines transforms with associated probabilities. Selection uses cumulative thresholds against a uniform random float — O(1) lookup for up to 6 transforms:

```c
float r = rng_f();
int t = 0;
while (r > cum_prob[t]) t++;
/* Apply transform t */
```

### Log-Density Rendering

```c
float norm = logf(1.0f + (float)hits[r][c])
           / logf(1.0f + (float)max_hits);
```

Maps the wide dynamic range (1 hit to millions) to [0,1]. Four character levels: `. : + @`. The y-axis is flipped: grid row 0 corresponds to `y_max`, row ROWS-1 to `y_min` (IFS attractors are defined in a coordinate system where y increases upward).

*Files: `fractal_random/barnsley.c`*

---

## 50. DLA Extended — fractal_random/diffusion_map.c

Diffusion-Limited Aggregation (DLA) with Eden mode toggle and age-gradient coloring. Tip-enhancement biases growth toward the frontier's sharpest protrusions.

### Tip Enhancement

The stick probability for a walker adjacent to the aggregate depends on how many aggregate neighbours the candidate cell has. More aggregate neighbours → lower priority (interior) vs fewer neighbours → higher priority (tips). This produces denser branching than pure DLA.

### Eden Mode

Instead of simulating a random walk, Eden mode directly samples a random frontier cell (BFS queue of aggregate-adjacent empty cells) and adds it. This fills space uniformly, producing compact rounded shapes vs DLA's fractal branches. The toggle lets you switch mid-growth.

### Age Gradient

Each cell records its freeze tick as `uint16_t age`. Since age increases monotonically and wraps at 65536, old structures appear in one color and new growth in another — the structure's growth history is visible as a hue gradient.

*Files: `fractal_random/diffusion_map.c`*

---

## 51. DBM Tree — fractal_random/tree_la.c

Dielectric Breakdown Model: Laplace's equation controls fractal tree/lightning growth. Higher `η` exponent produces thinner, more branched structures.

### Gauss-Seidel Relaxation

The voltage field `φ` satisfies `∇²φ = 0`. Gauss-Seidel iterates until convergence (typically 50–200 passes per growth step depending on grid size):

```c
for (int r = 1; r < rows-1; r++)
    for (int c = 1; c < cols-1; c++)
        if (!aggregate[r][c])
            phi[r][c] = 0.25f*(phi[r-1][c]+phi[r+1][c]+phi[r][c-1]+phi[r][c+1]);
```

### φ^η Frontier Selection

Frontier cells are sampled with probability proportional to `phi[r][c]^eta`. Implementation: compute `sum = Σ phi^eta` over all frontier cells, then draw a random float in `[0, sum]` and walk the frontier until the cumulative weight exceeds the draw.

### Visual Encoding

`φ` value at each cell → color pair (high φ near electrode = bright; low φ deep in tree = dim). The gradient visualises the current electric field, showing where new growth is likely.

*Files: `fractal_random/tree_la.c`*

---

## 52. Lyapunov Fractal — fractal_random/lyapunov.c

Each pixel `(a, b)` in a 2D parameter space computes the Lyapunov exponent of the logistic map alternating between rates a and b.

### Progressive Row Rendering

Computing all pixels in one frame stalls the terminal for hundreds of milliseconds. Instead, `N_ROWS_PER_FRAME` rows are computed per frame, with the rest drawn from a cached `int8_t g_lya[ROWS][COLS]` buffer. Pressing `r` clears the buffer and restarts computation top-to-bottom.

### int8_t Bucket Encoding

The computed float λ (typically in `[-3, +3]`) is scaled by 32 and clamped to `[-127, +127]`, stored as `int8_t`. This is 4× more memory-efficient than float and enables fast sign-based palette lookup:

```c
int8_t bkt = (int8_t)clamp(lambda * 32.0f, -127.0f, 127.0f);
int pair, lvl;
if (bkt < 0) {                            /* stable: blue */
    lvl  = (int)((-bkt) * 3 / 128);
    pair = CP_S1 + lvl;
} else {                                  /* chaotic: red */
    lvl  = (int)(bkt * 3 / 128);
    pair = CP_C1 + lvl;
}
```

### Axis Orientation

The b-axis is inverted: row 0 = `b_max`, row ROWS-1 = `b_min`. This matches the standard Lyapunov fractal orientation where the "Zircon City" structure appears in the upper-left quadrant.

*Files: `fractal_random/lyapunov.c`*

---

## 53. Ant Colony Optimization — artistic/ant_colony.c

Stigmergic pathfinding on an 8-directional grid. Ants deposit pheromone on traversed cells; shorter paths accumulate stronger trails before evaporation erases them. The colony converges to the near-optimal path without central coordination.

### Pheromone Dynamics

Each tick: `τ ← τ × (1 − ρ) + Δτ`. Every ant that uses a cell deposits `Q / L_k` pheromone proportional to how short its total path was. The evaporation rate `ρ ≈ 0.1` prevents runaway accumulation on early suboptimal paths.

### Probabilistic Movement

An ant at cell i chooses neighbour j with probability `P(i→j) ∝ τ_ij^α × η_ij^β`, where `η_ij = 1/distance` is a heuristic and `α, β` balance exploitation vs. exploration. The implementation uses 8-directional grid movement (not a graph), making it visual but less rigorous than classical graph-based ACO.

The pheromone field `g_ph[GRID_H][GRID_W]` (float, ~128 KB) is the colony's shared memory. Two food sources and one nest define the pathfinding goal. O(N_ants × grid_area) per tick.

*Files: `artistic/ant_colony.c`*

---

## 54. Dune Rocket — artistic/dune_rocket.c

Homing missile simulation set in the Dune universe. Harkonnen rockets launch from a ship at the top of the screen and steer toward Arrakis ground targets using proportional navigation, each leaving a 30-position ring-buffer trail.

### Proportional Navigation

Each tick the rocket's velocity direction is bent toward the target by `TURN_RATE` radians per second: the velocity vector is rotated by `arctan2(cross, dot)` clamped to the turn budget. A sinusoidal lateral wobble `WOBBLE_AMP × sin(WOBBLE_FREQ × t)` is added for visual realism. Speed ramps from `ROCKET_SPEED0` to `ROCKET_SPEEDMAX` (acceleration phase).

### Trail and Explosion

The trail is a fixed-length ring buffer of past positions drawn at decreasing brightness (motion blur). On impact, ballistic explosion sparks are spawned: simple particles with gravity integrated by explicit Euler. Rocket orientation is mapped to one of eight ASCII direction glyphs (`▲ ↗ → ↘ ▼ ↙ ← ↖`).

*Files: `artistic/dune_rocket.c`*

---

## 55. Dune Sandworm — artistic/dune_sandworm.c

Arrakis sandworm simulation using a segmented-chain (Conga) follower model. The head drives along a sinusoidal underground path; each of N_SEGS=50 body segments follows its predecessor when separation exceeds SEG_LEN.

### Body Chain Kinematics

Underground path: `y = surface_row + SWIM_DEPTH + SWIM_AMP × sin(SWIM_FREQ × x)`. Breach arc: parabolic trajectory above the surface reaching BREACH_HEIGHT rows at the apex. After each breach, sand ripples expand radially at RIPPLE_SPEED cols/s. The terrain surface is a procedural height map from summed low-frequency cosine waves.

### Rendering

Segment direction determines the ASCII character: `─ │ ╱ ╲` for horizontal, vertical, or diagonal movement. The head displays a multi-row open-mouth sprite during breach. Up to MAX_WORMS=8 independent worms share the same screen.

*Files: `artistic/dune_sandworm.c`*

---

## 56. FFT Visualiser — artistic/fft_vis.c

Side-by-side display of a composite sine signal (top panel) and its frequency-domain magnitude spectrum (bottom panel), computed with a Cooley-Tukey radix-2 DIT FFT.

### Cooley-Tukey Butterfly

N_FFT=128 (power of 2). The iterative bottom-up butterfly avoids O(log N) stack depth. Bit-reversal permutation reorders samples in-place before the `log₂(N)` butterfly stages. Each stage doubles the butterfly span m and rotates by twiddle factor `W_m = exp(−2πi/m)`. Complexity O(N log N) vs. O(N²) naive DFT.

### Display

Three user-tunable sine components (frequency, amplitude, on/off toggle) are summed to build the signal each frame. Magnitudes `|X[k]| / (N/2)` for k = 0…N/2−1 are displayed as a bar chart; the N/2 bins above Nyquist are symmetric conjugates and are omitted. A white-noise burst can be injected to show broadband excitation.

*Files: `artistic/fft_vis.c`*

---

## 57. Fourier Draw — artistic/fourier_draw.c

DFT epicycle reconstruction: any closed curve is decomposed into rotating arms via the Discrete Fourier Transform, then reconstructed by animating those arms in order of decreasing magnitude.

### DFT and Epicycle Sort

Shape samples `z[k] = x[k] + i·y[k]` are transformed with the O(N²) DFT (computed once per shape change). Coefficients are sorted by magnitude descending — largest circles first — so the partial sum converges quickly and the sort order is energy-optimal (Parseval greedy).

### Trail and Ghost Overlay

A ring buffer `g_trail` (length 512) stores the tip path of the last arm; this is the reconstructed shape gradually appearing. A ghost overlay `g_ghost` draws the original sample points for comparison. Cell-aspect correction scales x-coordinates before the DFT so the result appears undistorted on non-square terminal cells. A Parseval energy bar shows the cumulative power fraction of displayed arms.

*Files: `artistic/fourier_draw.c`*

---

## 58. Gear — artistic/gear.c

A rotating wireframe gear with physically correct tangential spark emission. The gear outline is computed analytically each frame from polar proximity tests — no mesh, no rasterization.

### Analytic Wireframe

Each terminal cell's center is tested against three geometric features in local polar coordinates: hub ring, spokes, and tooth sides. Proximity thresholds (THRESH_CIRC=7.5 px, THRESH_SIDE=4.0 px, THRESH_SPOKE=3.8 px) determine which feature is drawn. `line_char(angle)` maps the local tangent or radial direction to `- \ | /`. Tooth phase: `phase = fmod(ang_local × N_TEETH / 2π, 1.0)` with `TOOTH_DUTY=0.42`.

### Tangential Spark Physics

Emission rate scales with angular velocity: `total_rate = SPARK_BASE_RATE × (rot_speed / ROT_BASE) × spark_density`. At birth, `tang_vx = −sin(tip_ang) × rot_speed × R_OUTER × TANG_SCALE`. At high speed (20 rad/s) the tangential velocity (880 px/s) dominates the radial kick (35–120 px/s), producing dramatic spiral arcs. Drag uses the exact solution `vx *= exp(−DRAG × dt)`, which is frame-rate independent. Ten named color themes remap all nine spark color pairs simultaneously via `init_pair()`.

*Files: `artistic/gear.c`*

---

## 59. Graph Search — artistic/graph_search.c

Three graph search algorithms (BFS, DFS, A*) animated step-by-step on a 40-node planar-ish random graph laid out with the Fruchterman-Reingold spring-repulsion algorithm.

### Layout and Algorithms

Force-directed layout: nodes repel with force ∝ k²/d and edges attract with force ∝ d²/k, where k = √(area/N). BFS uses a queue (shortest path by hop count, O(V+E)); DFS uses a stack (finds a path but not necessarily shortest, O(V+E)); A* uses a min-heap with Euclidean heuristic h(n) (admissible, optimal, O((V+E) log V)).

Source and goal are chosen as the farthest-apart pair. Node colors encode state: white=unvisited, cyan=frontier, dark-grey=visited, green=source, red=goal, yellow=final path.

*Files: `artistic/graph_search.c`*

---

## 60. Hex Life — artistic/hex_life.c

Conway's Game of Life adapted to a hexagonal grid. Each cell has 6 neighbors (offset-row layout) rather than 8. Rule B2/S34 produces qualitatively different behavior — more isotropic growth, different oscillators and gliders.

### Hex Neighbor Lookup

Two neighbor offset tables, one per row parity (even/odd offset rows). Double-buffering: `g_grid` and `g_next` store current and next generation; `g_age` and `g_anext` track how many consecutive generations a cell has been alive. Age drives a color gradient: newborn→yellow, young→cyan, mature→teal, elder→dark.

### Rule Application

For each cell, count 6 neighbors. Apply: dead cell with exactly 2 live neighbors → born (B2); live cell with 3 or 4 live neighbors → survives (S34). The 6-neighbor symmetry eliminates the diagonal artifacts of square-grid CAs and makes boundary patterns more isotropic.

*Files: `artistic/hex_life.c`*

---

## 61. Jellyfish — artistic/jellyfish.c

Physics-based bioluminescent jellyfish with a four-state locomotion model. Up to 8 jellies drift upward through the terminal using jet propulsion driven by a state machine.

### Four-State Physics

`IDLE → CONTRACT → GLIDE → EXPAND → IDLE`. IDLE: bell fully open, passive sink under GRAVITY=30 px/s². CONTRACT (0.19 s, fast ease-out): bell squeezes to 42% width; at completion `vy = −JET_THRUST=190 px/s` burst upward. GLIDE (0.38 s): bell held closed, `vy *= exp(−DRAG_VY × dt)` exact exponential decay. EXPAND (0.68 s, smoothstep): bell re-opens.

### Bell Rendering

Two independently controlled axes: `head_f` (horizontal width, remapped from `bell_open` to [0.42, 1.0]) and `crown_f` (apex height, `0.88 + 0.12 × crown_f` limit). Scan-line ellipse renders the upper half-ellipse with five row zones: closing roof, rounded crown, sides, lower body, rim. Three tentacles (N_TENTACLES=3, TENTACLE_SEGS=20) hang below using a travelling sine wave gated by `wave_scale`; velocity lag `−vy × TENT_LAG × depth` streams tentacles behind fast motion.

*Files: `artistic/jellyfish.c`*

---

## 62. Network SIR Simulation — artistic/network_sim.c

SIR epidemic model on a Watts-Strogatz small-world network. Split display: left panel shows N=40 nodes colored by state (S/I/R) on a ring layout; right panel shows a scrolling stacked epidemic curve over time.

### Watts-Strogatz Construction

Start with a K=4 ring graph; rewire each edge with probability p=0.15 to a random node. This creates the small-world property: short average path length and high clustering coefficient. Rewired shortcut edges are highlighted when infection spreads through them.

### SIR Dynamics

Per tick: each infected node infects each susceptible neighbor with probability β; each infected node recovers with probability γ. Basic reproduction number R₀ = β·⟨k⟩/γ — epidemic spreads when R₀ > 1. Arrow keys adjust β and γ live; the epidemic curve reacts immediately.

*Files: `artistic/network_sim.c`*

---

## 63. Railway Map — artistic/railwaymap.c

Procedural transit map generator. Lines are built from five path templates (H_FULL, V_FULL, Z_SHAPE, REV_Z, DOUBLE_Z) routed through an 8×6 logical grid. Stations emerge at every grid node; nodes shared by two or more lines become interchange stations rendered as `O`.

### Canvas Rendering

A flat 2D canvas of cell descriptors records which H-line and V-line pass through each terminal cell. At render time: 0 lines → space, 1 H-line → ACS_HLINE, 1 V-line → ACS_VLINE, both → ACS_PLUS. Grid node → terminal cell mapping uses linear interpolation between margin bounds so the map fills any terminal size. Station names are placed perpendicular to the line direction.

Animated trains are a pool of (line, progress) structs; progress advances along the line's waypoint path each tick. New maps are generated on demand with `r`.

*Files: `artistic/railwaymap.c`*

---

## 64. Sand Art — artistic/sand_art.c

Falling-sand cellular automaton inside a centred hourglass shape with flippable gravity. Five colored sand layers fill the top bulb and fall through the narrow neck into the bottom bulb.

### CA Rule and Geometry

Grid of `uint8_t` cells (0=empty, 1–5=sand color). Each tick, cells are scanned away from the gravity direction (bottom-to-top for downward gravity) so each grain moves at most once per frame. Movement priority: forward (toward gravity) > diagonal-left > diagonal-right (diagonals randomised to avoid left-bias). A hourglass mask built at startup defines accessible cells; sand checks the mask before moving.

The angle of repose (~45°) emerges naturally from the diagonal movement rule — grains slide diagonally only when blocked directly. Pressing Space flips gravity direction; sand avalanches back through the neck.

*Files: `artistic/sand_art.c`*

---

## 65. X-Ray Swarm — artistic/xrayswarm.c

Up to 5 swarms of particles, each with a wandering queen and workers that shoot outward as long fading rays. Workers cycle through a 4-phase state machine: DIVERGE → PAUSE → CONVERGE → PAUSE → repeat.

### Trail and Phase Machine

Each worker carries a TRAIL_LEN=48 position ring buffer. Older trail positions are drawn at decreasing brightness, creating a long fading ray appearance. The heading is locked to the outward direction from the queen at the start of DIVERGE and held fixed during flight. Queen steering is an Ornstein-Uhlenbeck-like bounded random walk: each tick a small random angle δθ is added to heading, bounded to avoid tight circles.

Cell-aspect correction (CELL_W=8, CELL_H=16) ensures equal angular spacing on screen. Multiple swarms can be added/removed at runtime.

*Files: `artistic/xrayswarm.c`*

---

## 66. Shepherd Herding — flocking/shepherd.c

Interactive boid herding simulation. A flock of sheep uses Classic Boids rules (separation, alignment, cohesion) plus a flee force when the player-controlled shepherd comes within FLEE_RADIUS pixels.

### Boids Plus Flee

Flee force overrides cohesion so panicking sheep disperse rather than cluster. Sheep speed has two modes: cruise (SHEEP_SPEED) and flee (SHEEP_SPEED_FLEE ≈ 1.5× cruise). The shepherd moves at fixed speed (SHEPHERD_SPEED px/s) driven by held arrow keys. Both operate in isotropic pixel space with CELL_W=8, CELL_H=16 aspect correction.

Sheep characters are selected from an 8-direction glyph table (`o < > ^ v / \`) based on velocity heading; bold red when fleeing. An optional dashed circle around the shepherd shows the panic zone radius.

*Files: `flocking/shepherd.c`*

---

## 67. Excitable Medium — fluid/excitable.c

Greenberg-Hastings N-state cellular automaton modeling excitable media such as cardiac muscle. State 0 = resting; state 1 = excited; states 2..N-1 = refractory.

### Update Rule

Von Neumann neighborhood (4 orthogonal neighbors), periodic boundary. Rule: `if cell==0 AND any neighbour==1 → cell=1`; `if cell>0 → cell=(cell+1) % N`. The refractory period N−2 steps prevents a wave from re-exciting cells it just passed through, producing ordered ring waves. Spirals emerge from broken wave fronts: the open end curls into susceptible cells.

### Presets

Four initial conditions: SPIRAL (broken wave front curling into rotation), DOUBLE (two counter-rotating spirals), RINGS (radially periodic concentric targets), CHAOS (5% random seeding → turbulent multi-spiral). Double-buffering ensures each cell's next state depends only on current-tick neighbors. STEP_NS = 1/15 s, RENDER_NS = 1/30 s — CA runs at half display rate.

*Files: `fluid/excitable.c`*

---

## 68. SPH Fluid — fluid/fluid_sph.c

Smoothed Particle Hydrodynamics simulation. Particles carry mass, position, velocity, and pressure; density is estimated by summing a kernel function over neighbors. Five scene loaders (blob, rect, fountain, collision, rain) initialize different configurations.

### Kernel and Forces

Kernel: `w(d) = (d/H − 1)²` for d < H (SMOOTH_RADIUS=2.2), zero otherwise. Density: `ρᵢ = Σⱼ w(dᵢⱼ)`. Pressure force: `w(dᵢⱼ) × (ρᵢ + ρⱼ − 2ρ₀) × K_pressure / ρᵢ` — the sign of `(ρᵢ+ρⱼ−2ρ₀)` handles both repulsion (overcrowded) and cohesion (underdense) in one expression. Viscosity: `(vⱼ − vᵢ) × K_viscosity × w(dᵢⱼ)`. Integration: symplectic Euler (velocity before position) for better energy conservation.

### Spatial Hash Grid

GCELL=3 ≥ SMOOTH_RADIUS=2.2 ensures all neighbors within H lie in the 3×3 surrounding cells. Linked lists per cell (`g_ghead`, `g_gnext`) reduce neighbor search from O(N²) to O(N·k) where k≈15 average neighbors — approximately 50× speedup at N=600.

*Files: `fluid/fluid_sph.c`*

---

## 69. Lenia — fluid/lenia.c

Continuous cellular automaton generalizing Conway's Life to real-valued states and smooth ring kernels. Each step: convolve state grid with kernel K, apply growth function G, update state.

### Kernel and Growth

Ring kernel K: nonzero only in an annular ring at radius R, using the bump function `exp(4 − 4/(r(1−r)))`. Normalized so total weight = 1. Growth function: `G(u) = 2·exp(−((u−μ)/σ)²) − 1` (a Gaussian centered at μ). Update: `A(x, t+dt) = clamp(A(x,t) + dt·G(U(x,t)), 0, 1)`.

The parameter pair (μ, σ) defines a "species." Preset creatures — Orbium (μ=0.15, σ=0.015, R=13), Aquarium (μ=0.26, σ=0.036, R=10), Scutium (μ=0.17, σ=0.015, R=8) — exhibit autonomous locomotion and division. The kernel is precomputed as a sparse list of (dr, dc, weight) entries after threshold pruning, stored in `g_kw`/`g_kdr`/`g_kdc`. Toroidal boundaries avoid edge artifacts.

*Files: `fluid/lenia.c`*

---

## 70. Marching Squares — fluid/marching_squares.c

2-D isosurface extraction (the 2-D analogue of Marching Cubes). Extracts contours from a metaball scalar field using a 16-case lookup table driven by 4-bit corner classification.

### Algorithm

For each 2×2 cell of grid corners: classify each corner as inside (f > threshold) or outside → 4-bit index → look up which edges the contour crosses → linearly interpolate crossing position along each edge → draw ASCII character. The 16 cases cover all possible inside/outside combinations; cases 0 and 15 produce no contour.

### Scalar Field

Metaball potential: `f(x,y) = Σ Aᵢ / ((x−cxᵢ)² + (y−cyᵢ)²)` — inverse-square wells from moving blob sources. The field is sampled at `(col × ASPECT, row)` with ASPECT=0.5 to correct for non-square terminal cells, producing circular blobs. Multiple iso-levels can be drawn simultaneously with different characters.

*Files: `fluid/marching_squares.c`*

---

## 71. Stable Fluids (Navier-Stokes) — fluid/navier_stokes.c

Jos Stam's "Stable Fluids" (SIGGRAPH 1999) on an N=80 grid. Operator-splitting separates diffusion, projection, and advection into individually unconditionally stable sub-steps.

### Pipeline

Per frame: (1) add forces/dye sources; (2) diffuse velocity with Gauss-Seidel (ITER=16 passes); (3) project to divergence-free field via Poisson solve (Gauss-Seidel again); (4) semi-Lagrangian advection (back-trace + bilinear interpolation); (5) project again. The projection enforces ∇·u=0 by computing a pressure correction field.

Semi-Lagrangian advection is first-order accurate but unconditionally stable for large dt — there is no CFL constraint. Arrow keys inject a velocity source at screen center; Space drops a dye blob. O(N²·ITER) per frame; at N=80, ITER=16: ~100k operations per step, trivial at 30 fps.

*Files: `fluid/navier_stokes.c`*

---

## 72. FitzHugh-Nagumo Reaction Wave — fluid/reaction_wave.c

Two-variable PDE on the terminal grid modeling cardiac action potentials. `u` is the fast membrane voltage activator; `v` is the slow recovery inhibitor.

### Equations and Stability

`du/dt = u − u³/3 − v + D·∇²u + I`; `dv/dt = ε·(u + a − b·v)`. Fixed point: u* ≈ −1.2, v* ≈ −0.625 (resting state). The Laplacian uses a 5-point stencil; explicit Euler requires STEPS_PER_FRAME=8 sub-steps to satisfy the CFL condition `FN_DT · FN_D / dx² < 0.25`. A suprathreshold stimulus kicks `u` upward; the refractory period (v recovery) prevents back-propagation, producing ordered ring waves.

Four presets: TARGET RINGS, DOUBLE (collision/annihilation), SPIRAL (rotating spiral from broken ring), PLANE WAVE (sweeping front). Unlike Gray-Scott, FitzHugh-Nagumo has a single nonlinear term (u³) making it analytically tractable.

*Files: `fluid/reaction_wave.c`*

---

## 73. 2-D Wave Equation — fluid/wave_2d.c

FDTD (Finite-Difference Time-Domain) solver for the scalar wave equation `∂²u/∂t² = c²·∇²u`. Uses explicit Euler with two alternating time-level grids.

### Discretization and Stability

Update: `u[t+1] = 2u[t] − u[t−1] + C²·(uE + uW + uN + uS − 4u[t])` where C² = (c·dt/h)² = 0.16. 2-D CFL condition: C·√2 < 1 → 0.566 < 1 (stable). Two time levels (t and t−1) are required and stored in alternating grids `g0` and `g1`.

Point sources (Space) emit Gaussian impulses; five persistent oscillating sources (keys 1–5) create standing-wave interference patterns. A sponge layer of BORDER_W cells at the edges damps outgoing waves with multiplicative damping — without it, boundary reflections create a "box mode" standing wave. Color maps: negative u → blue, zero → black, positive → white.

*Files: `fluid/wave_2d.c`*

---

## 74. Apollonian Gasket — fractal_random/apollonian.c

Recursive circle-packing via Descartes' Circle Theorem. Starting from three mutually tangent circles, the unique fourth tangent circle is found and the process recurses into each of the three new gaps.

### Descartes Theorem

`(k₁+k₂+k₃+k₄)² = 2(k₁²+k₂²+k₃²+k₄²)` where k = 1/radius (negative for the enclosing circle). Solution: `k₄ = k₁+k₂+k₃ ± 2√(k₁k₂+k₂k₃+k₃k₁)`. Center found via the complex Descartes formula: `z₄ = (k₁z₁+k₂z₂+k₃z₃ ± 2√(k₁k₂z₁z₂+…)) / k₄`. Starting pack k = (−1, 2, 2, 3) is the unique integer Apollonian gasket.

Recursion terminates when the resulting circle is smaller than one pixel on screen. Circle count grows as 3^depth; DEPTH_MAX=7 gives ~6500 circles. Circles are drawn progressively (one per frame) so the gasket builds up visually. Terminal aspect correction ASPECT_R scales the radius when converting to cell coordinates.

*Files: `fractal_random/apollonian.c`*

---

## 75. 2-D Cellular Automaton (LtL) — fractal_random/automaton_2d.c

Extended 2-D Cellular Automaton implementing Larger-than-Life (LtL). Generalizes Conway's Life to radius R > 1: the neighborhood count is the number of alive cells in a (2R+1)² square. Birth/survive thresholds are range-based rather than bitmasks; multi-state dying trails decay over N−1 generations.

### Prefix Sum Optimization

A 2-D summed area table makes each neighborhood count O(1) after O(W×H) table construction: `P[r][c] = grid[r][c] + P[r-1][c] + P[r][c-1] − P[r-1][c-1]`. Rectangle sum in O(1): `P[r1][c1] − P[r0-1][c1] − P[r1][c0-1] + P[r0-1][c0-1]`. At R=5: naïve 121 additions per cell vs. 1 lookup — 121× speedup.

Toroidal wrapping is implemented by padding the grid with R-wide copies on all sides before building the prefix sum, eliminating all per-cell boundary branches. A population history ring buffer `g_pop_hist[512]` is displayed as a time-series graph.

*Files: `fractal_random/automaton_2d.c`*

---

## 76. Bifurcation Diagram — fractal_random/bifurcation.c

Logistic map bifurcation diagram: `x_{n+1} = r·xₙ·(1−xₙ)`. Each screen column maps to a distinct r value. After WARMUP transient iterations, PLOT values are plotted as colored dots showing the attractor.

### Period-Doubling and Auto-Zoom

For r < 3: converges to a fixed point. At r ≈ 3.0: period-2 bifurcation. Each bifurcation point satisfies `r_{n+1} − r_n → 1/δ` where δ = 4.669… is Feigenbaum's universal constant. Onset of chaos at r ≈ 3.5699 (accumulation point). Auto-zoom scrolls r toward r∞ = 3.5699 each frame, continuously revealing self-similar copies of the diagram. Color encodes x value: blue (low) → cyan → green → yellow → red (high). O(W × (WARMUP + PLOT)) per diagram redraw; each column is independent.

*Files: `fractal_random/bifurcation.c`*

---

## 77. Burning Ship Fractal — fractal_random/burning_ship.c

Escape-time fractal identical to Mandelbrot except the real and imaginary parts are forced positive before each squaring: `z ← (|Re(z)| + i|Im(z)|)² + c`.

### Absolute-Value Fold

Expanding: `Re_new = Re(z)² − Im(z)² + Re(c)`, `Im_new = 2|Re(z)|·|Im(z)| + Im(c)`. The `|Im|` term forces the imaginary component positive after each step, breaking Mandelbrot's 4-fold symmetry and creating the characteristic "ship" shape with mast-like spires and downward-pointing flames. The most detailed region is near c ≈ −1.77 − 0.04i.

Smooth coloring via fractional escape count: `t = iter + 1 − log₂(log₂|z|)` removes the banding of integer escape counts. A fire palette maps escape speed to color: inside-set → black, slow escape → dark red, fast → yellow/white. Pan and zoom support with a fly-to preset.

*Files: `fractal_random/burning_ship.c`*

---

## 78. Dragon Curve — fractal_random/dragon_curve.c

Paper-folding Dragon Curve constructed by iterative right-folding. The turn sequence for generation n: `T_n = T_{n-1}  R  reverse_complement(T_{n-1})` where reverse_complement flips L↔R and reverses order.

### Growth and Rendering

After n folds: 2ⁿ−1 turns, 2ⁿ segments. Gen 13 → 8192 segments. The path never self-intersects (proven); four copies at 0°/90°/180°/270° tile the plane without overlap (rep-tile of order 4). Segments are drawn one per frame using turtle graphics — F=forward, R/L=turn. Color encodes position in the sequence (age of segment), producing a visual record of the folding hierarchy. Aspect correction: horizontal steps are scaled by CELL_W/CELL_H ≈ 0.5 to keep the curve isotropic on non-square terminal cells.

*Files: `fractal_random/dragon_curve.c`*

---

## 79. L-System — fractal_random/l_system.c

String-rewriting Lindenmayer system with turtle-graphics rendering. Each generation replaces every variable in the string with a production rule; the resulting string is executed by a turtle (F=forward, +=left, −=right, [=push, ]=pop).

### String Rewriting

Each rewrite step scans the current string and concatenates productions into a scratch buffer — O(|string| × max_rule_length) copies. String length grows exponentially: for the Dragon Curve rule the length doubles each generation. MAX_STR=1 MB covers Koch gen 6 (~490 K), Hilbert gen 6 (~490 K), Branching Plant gen 7. STACK_MAX=128 for branch nesting depth.

Eight presets: Koch, Sierpinski, Plant, Dragon, Hilbert, and others. A bounding-box pre-pass walks the turtle without drawing to find the extent, then scale and center. Aspect correction uses `(STEP_PX_COL, STEP_PX_ROW)` accounting for CELL_W/CELL_H to keep branches isotropic.

*Files: `fractal_random/l_system.c`*

---

## 80. Newton Fractal — fractal_random/newton_fractal.c

Per-pixel Newton's method for `f(z) = z⁴ − 1`. Each point in the complex plane is iterated `z ← z − f(z)/f′(z) = (3z⁴+1)/(4z³)` until convergence to one of the four roots (1, −1, i, −i).

### Basin Coloring

Color encodes which root converged to (four hues); brightness encodes convergence speed (fewer iterations → lighter). Slow convergence occurs near basin boundaries where f(z) ≈ 0, causing large Newton steps and chaotic basin-switching. Non-converging points (cycles or divergence) → black. Convergence tolerance TOL=1e-5, MAX_ITER=64. Pan/zoom with four presets that zoom to each root's basin boundary.

*Files: `fractal_random/newton_fractal.c`*

---

## 81. Perlin Landscape — fractal_random/perlin_landscape.c

Three-layer parallax landscape with fractal Brownian motion terrain. The three layers (far/mid/near) scroll at different speeds, creating an illusion of depth.

### Fractal Brownian Motion

Height: `h(x) = Σ_k noise(x·fᵏ) / Aᵏ` with frequency ratio f=2 and amplitude ratio A=0.5 over OCTAVES=6 octaves. Perlin noise uses pseudo-random gradient vectors at integer lattice points, dotted with offset vectors, then smoothly interpolated with the fade function `6t⁵−15t⁴+10t³` (C² continuous). Hurst exponent H=1 (standard fBm). The scrolling camera advances the noise time parameter, giving a fly-over effect.

Height bins map to terrain types (water/plains/hills/peaks). Stars are scattered deterministically in the upper sky using a hash of the column index. Painter's algorithm: sky → far → mid → near (each layer overwrites the previous).

*Files: `fractal_random/perlin_landscape.c`*

---

## 82. Strange Attractor — fractal_random/strange_attractor.c

Point-density rendering of 2-D chaotic attractors. A point is iterated for many steps under the attractor map; visit counts accumulate in a density grid, which is log-normalized and mapped to a nebula palette.

### Attractors and Density Rendering

Six named attractors (Clifford-A, de Jong-B, Marek-C, Svensson, Bedhead, Rampe). All use the Clifford family: `x' = sin(a·y)+c·cos(a·x)`, `y' = sin(b·x)+d·cos(b·y)` except Bedhead which uses its own formula. Log density coloring: `brightness ∝ log(1+count)/log(1+max_count)`. Without log scaling, the densest "spine" would dominate and outlying filaments would be invisible. ITERS_PER_FRAME iterations per tick accumulate into the density grid; the image converges over time.

*Files: `fractal_random/strange_attractor.c`*

---

## 83. Convex Hull — geometry/convex_hull.c

Two convex hull algorithms compared side-by-side on 40 random points: Graham scan (O(N log N)) and Jarvis march / gift-wrapping (O(N·h)).

### Graham Scan vs. Jarvis March

Graham scan: find lowest-rightmost pivot, sort other points by polar angle, sweep a stack popping whenever the cross product indicates a clockwise (non-left) turn: `(B−A)×(C−A) < 0`. Each point is pushed and popped at most once — O(N) sweep after O(N log N) sort. Jarvis march: from the leftmost point, repeatedly find the most counter-clockwise next point. Optimal when h ≪ N; worst case O(N²) when all points are on the hull.

Cross product: `(Ax−Ox)(By−Oy)−(Ay−Oy)(Bx−Ox)`. Points are drawn as dots; hull edges as lines; hull vertices highlighted as `O`.

*Files: `geometry/convex_hull.c`*

---

## 84. Hex Grid — geometry/hex_grid.c

Hexagonal character grid with offset-row layout (pointy-top hexagons). Characters are placed at hex centers; color is assigned by ring distance from the center hex, creating concentric color bands. Each character changes at an independent random rate.

### Layout and Ring Distance

Offset-row layout: odd rows shifted right by HEX_DX/2 columns. Ring distance is approximated by concentric ring assignment (converting to cube coordinates for the exact formula). Six themes (fire, matrix, ice, plasma, gold, mono) map ring indices to color pairs. The "breathing" effect — cells shimmering at different rates — is an emergent property of many independent random timers.

*Files: `geometry/hex_grid.c`*

---

## 85. Polar Grid — geometry/polar_grid.c

Polar/radial character grid: characters at the intersections of concentric rings and radial spokes. Each ring rotates at its own angular speed (alternating direction), creating a gear-like visual.

### Differential Rotation

Each ring has an independent angular velocity; adjacent rings alternate clockwise/counter-clockwise. Position: `col = cx + r·cos(θ + ω_ring·t)`, `row = cy + r·sin(θ + ω_ring·t) × (CELL_W/CELL_H)`. The CELL_H/CELL_W ≈ 2 correction produces circular rings despite non-square cells. Color assigned by ring index (concentric bands). Characters at each cell change independently with per-cell random timers.

*Files: `geometry/polar_grid.c`*

---

## 86. Rect Grid — geometry/rect_grid.c

Full-screen rectangular character grid with two independent sinusoidal color waves. Every terminal cell flips its character at a random rate; color is driven by two spatial sine waves traversing the grid.

### Wave Superposition

`wave1 = sin(k1·col + ω1·t)`, `wave2 = sin(k2·row + ω2·t)`. The superposition creates a moving moiré-like pattern. When k1 ≈ k2 but ω1 ≠ ω2, a slowly-moving beat envelope appears. Per-cell random character change is a Poisson process with mean rate 1/avg_interval. Color is computed analytically each frame from the two wave values — no accumulation. Six themes and four speed presets.

*Files: `geometry/rect_grid.c`*

---

## 87. Fireworks Rain — matrix_rain/fireworks_rain.c

Extends the fireworks rocket state machine with matrix-rain arc trails. Each spark carries a TRAIL_LEN=16 position history; the trail is rendered as shimmering matrix-rain characters following the exact parabolic arc.

### Trail History

Update order is critical: (1) shift `trail[1..]` = `trail[0..]`; (2) store current `(cx, cy)` into `trail[0]`; (3) advance physics. So `trail[0]` always holds the position from one tick ago — the actual arc, not the next position. `trail_fill` ramps from 0 to TRAIL_LEN as the particle moves, growing organically from the burst point.

### Physics Split

Two separate gravity constants solve a conflict: ROCKET_DRAG=9.8 decelerates rockets quickly so they explode at varied screen heights; GRAVITY=4.0 with per-particle variance `g = GRAVITY × (0.8 + rand × 0.4)` gives particles enough upward momentum for a circular burst shape. Removing the vertical squash (`vy = sin(angle)×speed` without ×0.5) produces a true circular burst. Five themes remap the seven spark color pairs; CP_WHITE (pair 8) is always white regardless of theme.

*Files: `matrix_rain/fireworks_rain.c`*

---

## 88. Pulsar Rain — matrix_rain/pulsar_rain.c

Rotating neutron star (pulsar) with N evenly-spaced matrix-rain beams sweeping from a central `@` core. Each beam leaves a 46° angular wake of fading characters.

### Wake Rendering

Wake slot k=0 is the bright leading edge; k=WAKE_LEN=16 is the dim tail. Each slot is WAKE_STEP=0.05 rad behind the previous. Per beam per frame: only WAKE_LEN+1=17 trig calls (one per wake slot direction vector `cw[k], sw[k]=sin(wa)×ASPECT`). Each of N_RADII=80 radial samples reuses those pre-computed values — 34 trig calls for 2 beams vs. 1360 without pre-computation.

Render interpolation: `draw_angle = beam_angle + spin × alpha` extrapolates the beam's exact sub-tick position. At N=12, adjacent wakes overlap by 16° producing a dense merged-corona effect. The core `@` is drawn last — always on top of all beams. Five themes (green/amber/blue/plasma/fire).

*Files: `matrix_rain/pulsar_rain.c`*

---

## 89. Sun Rain — matrix_rain/sun_rain.c

180 radial streams of matrix characters emanating from a central `@` at evenly spaced angles. Each stream has a staggered phase offset so beams appear at different distances from the core, producing a stochastic solar-wind field.

### Isotropic Coordinates

Stream direction angle `θ = k×π/90` for k=0…179. Position: `col = cx + r·cos_a`, `row = cy + r·sin_a` where `sin_a = sin(θ)×ASPECT` (ASPECT=0.45) is baked in at ray init — no per-position trig. Render interpolation: `draw_r_off = r_off + speed × alpha` for smooth 60 fps motion at 20 Hz physics. Characters shimmer by ~75% random replacement each tick. Five themes remap stream colors.

*Files: `matrix_rain/sun_rain.c`*

---

## 90. Maze — misc/maze.c

Recursive-backtracker DFS maze generation with animated BFS solve. A W×H grid of cells with 4-bit wall bitmasks (N=1, E=2, S=4, W=8) is carved into a perfect maze (spanning tree), then solved for the shortest path.

### Generation and Solve

DFS: maintain a stack of visited cells; pick a random unvisited neighbor, carve the wall between them (clear bit in current + set opposite bit in neighbor), push. Backtrack when no unvisited neighbors remain. GEN_STEPS=4 DFS steps per frame for animated generation.

BFS solve: flood-fill from entrance level-by-level; first reach of exit = shortest path. Parent array reconstructs the path backward. SOL_STEPS=16 BFS steps per frame for animated solve. Display: 2×2 terminal pixels per maze cell → (2W+1)×(2H+1) terminal cells total.

*Files: `misc/maze.c`*

---

## 91. Sort Visualiser — misc/sort_vis.c

Five sorting algorithms animated as a vertical bar chart: Bubble, Insertion, Selection, Quicksort (Lomuto), and Heapsort. Exactly one compare-or-swap is executed per tick.

### Coroutine-Style Iterators

Each algorithm is implemented as a state machine (struct + step function) that advances one operation per call, enabling the animation loop to run at user-controlled rate without threads. Color encodes operation state: grey=unsorted, yellow=comparing, red=just swapped, green=in final sorted position. N_ELEMS=48 bars. Complexity reference: Bubble/Insertion/Selection O(n²); Quicksort O(n log n) average (Lomuto partition: pivot=last element); Heapsort O(n log n) worst-case (max-heap built in O(n), extracted in O(n log n)).

*Files: `misc/sort_vis.c`*

---

## 92. Aspect Ratio Demo — ncurses_basics/aspect_ratio.c

Minimal demonstration of terminal cell-aspect correction. Draws a circle that appears visually circular despite non-square terminal cells.

Terminal cells are typically ~2× taller than wide. Drawing `x = r·cos(θ)`, `y = r·sin(θ)` directly produces an ellipse. The fix scales x by 2: `x = cx + radius × 2 × cos(angle)`. This is the foundation of all isotropic drawing in the project. The demo uses a simple spin loop with getch() to hold the display; it is deliberately minimal — no fixed timestep, no ncurses color.

*Files: `ncurses_basics/aspect_ratio.c`*

---

## 93. Lines and Cols Query — ncurses_basics/tst_lines_cols.c

Minimal ncurses demo: queries and displays terminal dimensions using the `LINES` and `COLS` globals set by `initscr()`.

After `initscr()`, ncurses sets the global `LINES` (row count) and `COLS` (column count) to the current terminal size. `getmaxyx(stdscr, rows, cols)` is the per-window equivalent and is safer in multi-window programs. The program prints the dimensions with `printw()`, calls `refresh()`, waits for a keypress, then exits with `endwin()`. The entire file is under 15 lines — it exists purely to demonstrate the query API.

*Files: `ncurses_basics/tst_lines_cols.c`*

---

## 94. 2-Stroke Engine — physics/2stroke.c

Cross-section animation of a 2-stroke internal combustion engine. The crank angle θ advances each tick via slider-crank kinematics; the piston, connecting rod, and crankshaft are redrawn each frame.

### Slider-Crank Kinematics

Piston position from crank center: `y_wrist = R·cos θ + √(L² − R²·sin²θ)` (exact, not approximated). Crank radius CRANK_R=4, connecting rod CONROD_L=9. The 2-stroke cycle phase is determined by θ: exhaust port uncovers at ~75°, transfer port at ~90°, BDC at 180°, ports close on compression stroke, TDC spark at 0°. Each phase is labeled and port visibility is toggled by comparing θ to port-open thresholds.

The HUD shows RPM; `]/[` keys adjust speed. Rendering uses ASCII line-drawing primitives for the cylinder walls, piston rectangle, and connecting rod line.

*Files: `physics/2stroke.c`*

---

## 95. Black Hole — physics/blackhole.c

Physically accurate Schwarzschild black hole ray tracer. A lensing table maps each screen pixel to its ray outcome (disk hit, horizon, escape) by integrating null geodesics backward in time with RK4 — computed once at startup, then used for fast per-frame rendering.

### Geodesic Integration

Null geodesic acceleration in 3-D Cartesian embedding: `d²pos/dλ² = −(3/2)·|pos×vel|²·pos/|pos|⁵`. RK4 with adaptive step size `ds = clamp(r×0.05, 0.003, 0.10)`. Each ray also tracks `min_r` (closest approach) — rays that grazed the photon sphere (r=1.5 r_s) produce the photon ring via `ring_b = exp(−(min_r−1.5)×2.4)`.

### Doppler and Accretion

Disk material orbits at Keplerian speed `v_orb = √(M/(2r))`. Relativistic Doppler beaming: `D = [(1+β)/(1−β)]^(3/2)`. At ISCO (r=3 r_s): D_max ≈ 6.8× (bright side) vs D_min ≈ 0.15× (dim side). Combined brightness: `D × g × radial_profile × texture` where g = √(1−1/r) is gravitational redshift. 11 named themes; camera distance is runtime-adjustable (triggers lensing rebuild).

*Files: `physics/blackhole.c`*

---

## 96. Bubble Chamber — physics/bubble_chamber.c

Charged particles in a uniform magnetic field (B perpendicular to screen) leaving spiral ionization tracks as they lose energy, mimicking real bubble chamber photographs.

### Exact Rotation Integration

Instead of Euler-approximating the Lorentz force, each step applies an exact 2-D rotation matrix `R(ω·dt)` to the velocity vector: `v_x' = vx·cos(ω)−vy·sin(ω)`, `v_y' = vx·sin(ω)+vy·cos(ω)`. This preserves orbital radius exactly (no energy drift). Ionization drag: `|v| *= (1−DRAG)` each step — orbits spiral inward as the particle slows.

Five particle types (e⁻, e⁺, μ, π, p) with different q/m ratios produce visually distinct curvatures. Each particle stores a ring buffer of TRAIL_LEN=300 positions drawn with age-faded characters (`O * + .`). The field direction can be flipped at runtime; field strength is adjustable.

*Files: `physics/bubble_chamber.c`*

---

## 97. Elastic Collision — physics/elastic_collision.c

Impulse-based elastic collision simulation. N_DISCS=25 colored discs bounce off each other and the walls with conservation of momentum and kinetic energy.

### Impulse Resolution

For each overlapping pair, a single impulse along the collision normal simultaneously satisfies non-penetration and elastic restitution: `J = 2·m₁·m₂/(m₁+m₂) · Δv·n̂`. Velocities updated: `v₁ -= J/m₁·n̂`, `v₂ += J/m₂·n̂`. Positional correction: `overlap = (r₁+r₂−d)/2` pushed along n̂. The check `dot(rel_v, n̂) >= 0` skips pairs that are already separating. Mass model: `mass = r²` (uniform-density 2-D disc). O(N²) pair checks per tick — acceptable for N=25.

*Files: `physics/elastic_collision.c`*

---

## 98. Gyroscope — physics/gyroscope.c

Free rigid body rotation with quaternion orientation tracking. Demonstrates the Dzhanibekov effect (intermediate-axis theorem): rotation near the middle principal axis is unstable.

### Euler's Equations and Quaternion Integration

State vector: [ωx, ωy, ωz] in body frame + [qw, qx, qy, qz] quaternion. Euler's equations (nonlinear): `I₁·dωx/dt = (I₂−I₃)·ωy·ωz`, etc. RK4 integration (SUB_STEPS=8 per tick) handles the stiffness. After each step, q is re-normalized: `q /= |q|`; the extracted axes are Gram-Schmidt re-orthogonalized to prevent floating-point drift.

The polhode tip (angular momentum vector in body frame) is drawn as a ring-buffer trail `g_app.scene.gyro.trail[TRAIL_LEN=300]`. Orthographic projection of the 3-D wireframe (ring + axes) to 2-D with ASPECT=CELL_W/CELL_H ≈ 0.5 for circular appearance.

*Files: `physics/gyroscope.c`*

---

## 99. Three-Body Orbit — physics/orbit_3body.c

Three equal masses under mutual Newtonian gravity, integrated with Velocity Verlet in natural units G=M=1. Defaults to the stable figure-8 choreography (Chenciner-Montgomery 2000).

### Figure-8 and Chaos

The figure-8 initial conditions were found by numerical minimization of the action integral. All three bodies share the same orbit offset by 1/3 of the period. DT=0.001 (natural time units); STEPS per frame controls simulation vs. display speed tradeoff. O(N²)=O(9) force pairs — trivial. Key `x` adds a random perturbation, breaking the symmetry and revealing the underlying chaos (sensitivity to initial conditions).

Trail color encodes speed: slow → blue, fast → white. Trails fade with age; TRAIL_LEN is user-adjustable at runtime. Zoom and pan supported.

*Files: `physics/orbit_3body.c`*

---

## 100. Pendulum Wave — physics/pendulum_wave.c

N=15 pendulums with lengths chosen so pendulum n completes (N_BASE + n) full oscillations in T_SYNC seconds, creating a synchrony-drift-resynchrony cycle.

### Analytic Integration

`θ_n(t) = amp × sin(ω_n × t)` — exact solution to the small-angle approximation `θ'' ≈ −(g/L)·θ`. No numerical integrator needed; each frame is one `sinf()` call per pendulum. Length-frequency relation: `L_n = g/ω_n²`. At t=0 all are in phase; over T_SYNC they drift through waves, spirals, and other Lissajous-like patterns, then "clap" back to perfect synchrony because each pendulum completes an integer number of full cycles in exactly T_SYNC.

Color cycles through the rainbow across the pendulum array. Each pendulum is drawn as a vertical string of `|` chars with a colored bob `O` at the displaced position.

*Files: `physics/pendulum_wave.c`*

---

## 101. Rigid Body — physics/rigid_body.c

2-D rigid-body simulation with AABB collision detection, Baumgarte positional correction, velocity impulse with restitution, and Coulomb friction. Supports cubes (AABB) and spheres (drawn as circles, physics as AABB).

### Two-Pass Resolution

Each solver iteration: Pass A — positional correction (always applied, even when separating): `corr = max(depth−SLOP, 0) × BAUMGARTE / (imA+imB)`. This fixes the critical bug where bodies spawning overlapping were never pushed apart. Pass B — velocity impulse (only when approaching): `j = (1+e_eff)×vn / (imA+imB)` with micro-bounce suppression `e_eff=0` below REST_THRESH.

Floor and walls use full snap + impulse with no fraction. Sleeping: SLEEP_FRAMES quiet frames → frozen; woken by impulse or positional correction > WAKE_IMP. AABB for spheres: `hw=r, hh=2r` matches the drawn circle's screen extent (terminal cells are ~2× taller than wide).

*Files: `physics/rigid_body.c`*

---

## 102. Schrödinger Equation — physics/schrodinger.c

1-D quantum wavefunction evolution under the time-dependent Schrödinger equation, discretized with the Crank-Nicolson scheme (unconditionally stable, 2nd-order in time).

### Crank-Nicolson and Thomas Algorithm

`(I + iΔt/2·H)·ψ(t+Δt) = (I − iΔt/2·H)·ψ(t)`. The Hamiltonian `H = −½d²/dx² + V(x)` (natural units ℏ=m=1) discretized with a 5-point stencil produces a tridiagonal complex linear system solved with the Thomas algorithm O(N) per step — not O(N³). N_GRID=512, STEPS_PER_FRAME=20 sub-steps: ~10240 complex multiplications per frame, easily 30 fps.

Four presets: free Gaussian packet, finite barrier (quantum tunneling), harmonic well (bouncing packet), double slit (interference). Display: blue=Re(ψ), red=Im(ψ), white=|ψ|² (probability density). Bottom row shows the potential V(x).

*Files: `physics/schrodinger.c`*

---

## 103. Soft Body (PBD) — physics/soft_body.c

Position-Based Dynamics soft-body simulation. Multiple soft cubes and spheres with full pairwise collision, handled by a generic boundary-polygon test covering all shape combinations.

### PBD Constraint Projection

Verlet predict → project distance constraints × PBD_ITERS=6 → clamp walls. Constraints: structural (keep nodes at rest distance, STRUCT_K=1.0), shear (diagonal links, SHEAR_K=0.8), volume (area preservation). PBD is unconditionally stable; stiffness is controlled by iteration count, not spring constant.

Inter-body collision: for each node of body A penetrating body B's convex hull, push A's node outward by depth/2 along nearest-edge normal, and push B's two nearest boundary nodes inward by depth/4 each (Newton's 3rd law in position space). COLL_ITERS=2 collision passes per physics step.

*Files: `physics/soft_body.c`*

---

## 104. Mandelbulb Rasterizer — raster/mandelbulb_raster.c

Hybrid mesh-based Mandelbulb rendering. The fractal surface is tessellated into a triangle mesh at startup via sphere projection, then rendered through the standard software rasterization pipeline each frame.

### Sphere Projection Tessellation

For each (lat θ, lon φ) point on a UV sphere (NLAT=28, NLON=56), march inward from r=1.5 until the Mandelbulb SDF falls below MB_HIT_EPS — the first crossing is a surface vertex. Valid adjacent vertices are connected into quads → 2 triangles. This captures only the outermost surface shell; concave cavities and re-entrant surfaces are invisible. The tessellation runs in O(NLAT×NLON×MAX_MARCH); NLON=2×NLAT keeps UV cell aspect approximately square.

### Fragment Shaders

Three modes: `frag_phong_hue` (Blinn-Phong + HSV color from smooth escape time), `frag_normals` (surface normal → azimuth hue + Y-component brightness), `frag_depth_hue` (pure escape-time → rainbow hue). Full-color framebuffer using a 12-hue terminal color pair set; luminance drives the Bourke ASCII density ramp with Bayer dithering.

*Files: `raster/mandelbulb_raster.c`*

---

## 105. Mandelbulb Explorer — raymarcher/mandelbulb_explorer.c

3-D Mandelbulb fractal rendered by sphere marching a distance estimator (DE). Each pixel follows a null-geodesic-style march until the DE falls below HIT_EPS, then shading is computed using SDF-gradient normals, ambient occlusion, and soft shadows.

### Distance Estimator

Mandelbulb DE (Iñigo Quílez formula): iterate `z ← z^p + c` in spherical coordinates tracking derivative `dr = p·r^(p-1)·dr + 1`. Return `0.5·log(r)·r/dr`. Smooth escape count: `μ = iter + 1 − log(log(|z|)/log(bail))/log(power)` removes hard color bands. The power parameter p is runtime-adjustable (default p=8); changing it rebuilds nothing — the march re-evaluates the DE each frame.

### Shading

Ambient occlusion: sample the DE at several offsets along the normal and compare to expected distances — closer-than-expected → occluded. Soft shadows: march from the hit point toward each light; if the march passes close to geometry, the shadow is soft. A rotating camera orbits the bulb; multiple themes map escape time to palettes.

*Files: `raymarcher/mandelbulb_explorer.c`*

---

## 106. SDF Gallery — raymarcher/sdf_gallery.c

Five raymarched scenes demonstrating SDF composition techniques: smooth union, boolean CSG, twist deformation, domain repetition, and organic sculpting.

### SDF Operations

Smooth union: `smin(d1, d2, k)` — blends two SDFs into a connected surface with controllable blend radius k. Boolean CSG: `union=min(d1,d2)`, `intersection=max(d1,d2)`, `subtraction=max(d1,−d2)` — exact set operations. Twist: rotate input point `p.yz` by angle ∝ p.x before evaluating the SDF; domain deformation violates the Lipschitz condition so a relaxation factor (0.5) prevents overshoot. Domain repetition: `p = mod(p, cell_size) − cell_size/2` produces an infinite periodic field from a single primitive evaluation.

Progressive render: the framebuffer is filled over multiple frames; partially computed frames are displayed immediately. Features: soft shadows, ambient occlusion, three-point lighting, five themes, three lighting modes.

*Files: `raymarcher/sdf_gallery.c`*

---

## 107. Capsule Raytrace — raytracing/capsule_raytrace.c

Analytic ray-capsule intersection using the Íñigo Quílez decomposition: cylinder body test (quadratic after projecting out the axial component) followed by hemisphere cap tests (sphere quadratics) at each endpoint.

### Intersection Algorithm

Let `ba = cb − ca` (axis), `oa = ro − ca`. Project out axial component: `a = |ba|² − (ba·rd)²`, `b = |ba|²·(rd·oa)−(ba·oa)·(ba·rd)`, `c = |ba|²·(|oa|²−r²)−(ba·oa)²`. Discriminant `h = b²−a·c`. Body hit if `t > ε` and axial position `y = (ba·oa) + t·(ba·rd) ∈ (0, |ba|²)`. If body misses (y out of range), try cap sphere at `ca` or `cb` using `oc = oa` or `oc = oa − ba`. If `h < 0` (infinite cylinder missed), skip both caps.

Body normal: strip axial component from `(P − ca)`. Cap normal: `normalize(oc + t·rd)`. Normal transition at the seam is C¹ continuous. Four rendering modes (Phong, normals, Fresnel, depth); six themes.

*Files: `raytracing/capsule_raytrace.c`*

---

## 108. Cube Raytrace — raytracing/cube_raytrace.c

Analytic ray-AABB intersection using the slab method. The cube tumbles via a rotation matrix applied to the ray (inverse transform), not to the geometry.

### Slab Method

For each axis: compute entry/exit times `t_near_i = (−s−ro_i)/rd_i`, `t_far_i = (s−ro_i)/rd_i`. Overall entry = max of entries, overall exit = min of exits. Hit if `t_near ≤ t_far` and `t_far > 0`. The axis that produced `t_near` (recorded during the loop as `near_ax`) is the face the ray actually hit — exact, no post-hoc inference. Normal sign: `(rd_i > 0) ? −1 : +1`.

### Wireframe Mode

After intersection, the two "free" coordinates on the hit face give edge distance: `min(s−|u|, s−|v|)/s`. If below WIRE_THRESH=0.055, draw edge; otherwise skip (interior transparent). Produces pixel-perfect lines analytically — no rasterization artifacts. Four rendering modes (Phong, normals, wireframe, depth); six themes.

*Files: `raytracing/cube_raytrace.c`*

---

## 109. Path Tracer — raytracing/path_tracer.c

Unidirectional Monte Carlo path tracer rendering a Cornell Box. Each frame adds SPP samples to a per-pixel accumulator; the displayed image is `accumulator / sample_count`, converging from noisy to clean over ~512 samples.

### Path Tracing Algorithm

For each pixel: cast a ray; at each surface hit, sample a random new direction from the cosine-weighted hemisphere around the surface normal (`θ = arccos(√(1−u))`, `φ = 2π·v`). Accumulate emitted radiance. Russian roulette termination: at each bounce, terminate with probability `1 − max_color_component`; scale surviving rays to preserve unbiasedness. Cosine-weighted sampling importance-samples the Lambertian BRDF `f_r = ρ/π`, cancelling the cosine factor for simpler accumulation.

Reinhard tone mapping: `L_out = L/(1+L)`. 216-color xterm cube palette; luminance → Bourke ASCII density ramp. Scene: red/green/white Cornell walls, warm overhead light, two colored spheres.

*Files: `raytracing/path_tracer.c`*

---

## 110. Sphere Raytrace — raytracing/sphere_raytrace.c

Analytic ray-sphere intersection. Each terminal cell fires one ray; the quadratic ray-sphere equation is solved exactly. Three-point lighting (warm key, cool fill, bright rim) and four rendering modes.

### Quadratic Intersection

`a=|rd|²=1`, `b=2(rd·oc)` (oc = ro − center), `c=|oc|²−R²`. Discriminant `b²−4c`; miss if negative. Surface normal: `N = (P − center) / R`. Phong: `I = ka + kd·max(N·L,0) + ks·max(R_v·V,0)^shininess`. Fresnel (Schlick): `F = F0 + (1−F0)·(1−N·V)^5`. Six themes (gold, ice, crimson, emerald, amethyst, neon). Four modes: Phong, normals, Fresnel, depth. Camera orbits the sphere; zoom adjustable at runtime.

*Files: `raytracing/sphere_raytrace.c`*

---

## 111. Torus Raytrace — raytracing/torus_raytrace.c

Analytic ray-torus intersection via quartic polynomial root-finding. The torus lies in the XZ plane; substituting the ray into `(√(x²+z²)−R)² + y² = r²` yields a degree-4 polynomial in t.

### Quartic Solver

Coefficients A, B, C, D are derived by algebra from the torus equation. Rather than the unstable Ferrari formula, roots are found by evaluating the polynomial at sample points, then bisecting sign-change intervals to locate each real root. The minimum positive root is the front surface hit. Surface normal at hit point p: `N = normalize(p − R·normalize(p.xz × (0,1)))` (strips the Y component and points radially from the nearest ring axis point).

Same inverse-ray-transform approach as cube and capsule: rotate the ray into object space, intersect the axis-aligned torus, transform the normal back. Four rendering modes (Phong, normals, Fresnel, depth); six themes (titanium, solar, cobalt, forest, rose, chrome).

*Files: `raytracing/torus_raytrace.c`*

---

## 112. Hexapod Tripod Walker — animation/hexpod_tripod.c

Six-legged robot that walks across the terminal in any direction using a **tripod gait** and **per-leg 2-joint analytical IK**. The body is a rigid rectangle; the robot steers via arrow keys with gradual angular interpolation, and wraps toroidally at all four screen edges.

### Tripod Gait Architecture

Six legs are statically assigned to two interlocked groups:

- **Group A** = {0 left-front, 3 right-mid, 4 left-rear}
- **Group B** = {1 right-front, 2 left-mid, 5 right-rear}

Each group forms a support triangle. While one triangle swings, the other three planted feet hold the body stable — the robot never has fewer than three ground contacts. The transition FSM requires two simultaneous conditions before switching groups: `phase_timer ≥ PHASE_DURATION` AND every leg in the current group has landed. This prevents switching while a foot is still in the air at slow speeds where `PHASE_DURATION` expires before `STEP_DURATION`.

Each swinging foot follows a parabolic arc: horizontal position uses `smoothstep` ease between `foot_old` and `step_target`; vertical offset uses `−STEP_HEIGHT × sin(π × step_t)` with the raw (un-eased) `step_t`, giving a symmetric bell curve rather than a front-weighted or back-weighted lift.

### 2-Joint Analytical IK Pipeline

Given hip H and foot target T:

1. `dist = clamp(|T−H|, |U−L|+1, U+L−1)` — clamp to reachable range
2. `base = atan2(T.y−H.y, T.x−H.x)` — direction to target
3. Law of cosines: `cos_h = (dist² + U² − L²) / (2·dist·U)`, `ah = acos(cos_h)`
4. Left legs (even index): `knee_angle = base − ah` → knee bends toward −y (outward up)
5. Right legs (odd index): `knee_angle = base + ah` → knee bends toward +y (outward down)

The sign convention is **opposite** to `ik_spider.c`. In the spider the hips extend along a sinusoidal heading so the knee must flip around the heading vector. In the hexapod the hips extend perpendicular to the body axis (±y), so the signs are reversed for correct outward knee bend.

### Heading-Based Rotation System

All hip attachment positions and rest foot targets are defined in body-local space (body faces +x). `rotate2d(v, heading)` transforms them to world space each tick. This means the same static tables `HIP_LOCAL_X/Y` and `REST_FORWARD/SIDE` work for any heading — no per-direction special cases. The render layer performs a short-arc heading lerp between `prev_heading` and `heading` using the same ±π normalization trick to avoid a visual snap when heading crosses ±π.

### Stretch-Snap Mechanism

After a toroidal edge wrap or rapid turn, the hip can teleport away from a planted foot. If `|hip − foot| > UPPER_LEN + LOWER_LEN − 2`, the IK solver would receive an out-of-reach target and clamp artificially. The stretch-snap check runs after every hip recompute: it teleports the foot to `rest_target()` immediately (no swing animation) so the solver always receives a valid target. This prevents the one-frame "stretched leg" artifact.

### Sub-tick Alpha Interpolation

Physics runs at 60 Hz in a fixed-step accumulator. Between ticks, `alpha = sim_accum / tick_ns` linearly interpolates all positions (`prev_*` → current) before drawing. Heading uses short-arc lerp: `bh = prev_heading + normalize(heading − prev_heading) × alpha`. This decouples render frame rate from physics rate, producing smooth motion even at low sim Hz settings.

*Files: `animation/hexpod_tripod.c`*

---

## 112. Beam Bending & Vibration — physics/beam_bending.c

### Static Analysis — Euler-Bernoulli Beam

The Euler-Bernoulli beam equation `EI·w'''' = q(x)` is solved analytically for each combination of boundary condition and load type. Nine combinations are supported (3 BC types × 3 load types):

| BC | Load | Deflection w(x) |
|----|------|-----------------|
| Simply supported | center point | `Pbx(L²−b²−x²)/(6EIL)` for x≤a |
| Simply supported | distributed | `qx(L³−2Lx²+x³)/(24EI)` |
| Simply supported | end moment | `Mx(L²−x²)/(6EIL)` (approx as end point) |
| Cantilever | tip point | `Px²(3L−x)/(6EI)` |
| Cantilever | distributed | `qx²(6L²−4Lx+x²)/(24EI)` |
| Cantilever | base moment | `Mx²/(2EI)` |
| Fixed-fixed | center point | `Px²(3L−4x)/(48EI)` for x≤L/2 |
| Fixed-fixed | distributed | `qx²(L−x)²/(24EI)` |
| Fixed-fixed | end moment | moment-balanced as fixed-free |

The bending moment M(x) = EI·w'' is derived from the second derivative of each formula. Both static quantities are computed once per `init_beam()` call into arrays `b.w[]` and `b.M[]` of N_NODES=200 elements.

### Load Animation and Visualization

The load ramps in over `RAMP_SPEED=0.7s` via `load_anim` ∈ [0,1], which scales the deflection and moment during drawing. An exaggeration factor `b.exag` (keys e/E) multiplies the displayed deflection so hairline-thin deflections become visible.

**Curvature shading:** Each node selects a character from the density ramp `".,-~=+*#@"` based on `|w[i]|/max_deflection`, giving darker chars where curvature is highest. A 3-row band (`BEAM_HEIGHT=3`) is drawn per node; the centre row shows the deflection char, the outer rows show `.` for structural thickness.

**Bending moment panel:** A side panel of width `PANEL_W=22` draws a vertical bar for each node position proportional to `M[i]/max_moment`. Positive moments (sagging) draw downward with cyan; negative (hogging) draw upward with magenta.

### Dynamic Modal Superposition

Pressing `d` triggers the dynamic mode (`DynBeam.active=true`). Modal properties are computed for the current BC type:

- **Cantilever eigenvalues:** β₁L = 1.8751, 4.6941, 7.8548, 10.9955. Eigenfunctions involve `cosh`/`sinh`/`cos`/`sin` combinations requiring double precision to avoid cancellation.
- **Fixed-fixed eigenvalues:** β_nL = nπ (n=1,2,3,4). Pure sine eigenfunctions, numerically clean.
- **Simply supported eigenvalues:** β_nL = nπ. Pure sine eigenfunctions.

Modal mass and generalised force are computed by integrating `φ_n(x)·q(x)` over the beam length. Each mode then runs as an independent damped oscillator:

```
ωd = ω_n · √(1 − ζ²)         where ζ = MODAL_DAMP = 0.025
e  = exp(−ζ·ω_n·dt)
q(t+dt) = q_s + e·(A·cos(ωd·dt) + B·sin(ωd·dt))
```

The exact transition matrix is unconditionally stable — no step size constraint, unlike explicit Euler which becomes unstable when `ω_n·dt > 2`.

The dynamic deflection is `w(x) = Σ_n q_n(t)·φ_n(x)·exag`, added on top of zero (the modes are a complete basis; the static solution is embedded in the initial conditions of q and qdot).

*Files: `physics/beam_bending.c`*

---

## 113. Differential Drive Robot — robots/diff_drive_robot.c

### Kinematic Model

A differential drive robot has two independently driven wheels on a common axle. The pose is (x, y, θ) in 2D. For wheel velocities vL and vR with axle width L:

```
v     = (vL + vR) / 2          linear velocity
ω     = (vR − vL) / L          angular velocity (CW positive in y-down)
x'    = v · cos(θ)
y'    = v · sin(θ)
θ'    = ω
```

When `ω = 0` the robot moves straight. When `vL = −vR` the robot spins in place (R=0). The turn radius `R = v/ω` is displayed in the HUD (shown as `INF` when |ω| < 0.001).

The nonholonomic constraint — the robot cannot move sideways — is embedded in the kinematic equations: velocity is always along the heading vector (cosθ, sinθ). No penalty term is needed; the constraint is geometric.

### Input Mapping and Momentum

User input sets `v_cmd` (forward/reverse) and `w_cmd` (turn rate) via arrow keys. These are converted to individual wheel speeds:

```
vL = v_cmd − w_cmd · axle/2
vR = v_cmd + w_cmd · axle/2
```

Linear momentum: `V_DECAY = 1.00` — no automatic slowdown. The robot holds its last commanded speed indefinitely until the user brakes (`S` key sets v_cmd = w_cmd = 0) or steers. This models a real wheeled robot which keeps rolling unless braked.

Angular decay: `W_DECAY = 0.88` per tick — turns damp out quickly when the turn key is released, preventing endless spinning.

### Pixel-Space Physics and Aspect Correction

All positions are in pixel space (`CELL_W=8`, `CELL_H=16`). The axle length is defined in pixels (`AXLE_PX=36`). Wheel positions in pixel space:

```
left  wheel: (px + sin(θ)·half,   py − cos(θ)·half)   [north of body when facing east]
right wheel: (px − sin(θ)·half,   py + cos(θ)·half)   [south of body when facing east]
```

The sign convention uses y-down coordinates (standard terminal layout): θ=0 faces right (+x), θ=π/2 faces down (+y). Perpendicular to heading in y-down is (sinθ, −cosθ) for the left side.

Conversion to cell space (`px_to_cx/cy`) happens only at draw time. This keeps all velocity and distance arithmetic in consistent pixel units regardless of terminal size.

### Dot-Progression Arrow Drawing

The heading arrow and wheel velocity arrows use `draw_dot_line()` instead of Bresenham. The function steps in pixel space by `CELL_W` per sample and places characters based on fractional progress along the line:

```
t < 0.40 → '.'    (near, faint)
t < 0.75 → 'o'    (mid)
t ≥ 0.75 → '0'    (far, bold)
```

This avoids the `\`/`/` diagonal artifacts that appear when Bresenham steps diagonally through a character grid with a 2:1 cell aspect ratio. The caller places a cardinal tip char (`>`, `v`, `<`, `^`) or `o` for diagonal directions on top of the last dot.

### Trail Ring Buffer

The robot's centre traces a 600-slot ring buffer (`TRAIL_CAP=600`), sampled every `TRAIL_STEP=2` physics ticks. Trail dots fade from `.` (recent, bright) to `:` (old, dim) based on age fraction. Sub-tick render interpolation is applied to the live robot pose; trail dots use their stored pixel coordinates directly.

World-wrap interpolation suppression: if `|Δpx| > world_width/2` the interpolation delta is clamped to zero, preventing a ghost streak across the screen when the robot wraps a toroidal edge.

*Files: `robots/diff_drive_robot.c`*

---

## 114. Walking Robot — robots/walking_robot.c

A procedural bipedal walk cycle simulation combining sinusoidal forward kinematics for the swing phase with 2-joint analytical IK for the stance phase.

**Gait architecture:** The walk cycle drives joint angles via sinusoidal FK for each leg in swing. Foot contact locking freezes the grounded foot's world position so the body glides over it — this is the stance IK phase. The solver uses the standard 2-joint law-of-cosines formula to find hip and knee angles given the locked foot target.

**Body dynamics:** Body sway is computed as a lateral sinusoid locked to the step frequency. A separate vertical oscillation creates the characteristic human bounce. Centre-of-mass (COM) projection is drawn as a vertical line from the pelvis to the ground, giving immediate visual feedback on balance.

**Rendering:** Motion trails are stored in a ring buffer and rendered behind the figure using fading characters. A shadow ellipse is projected below the feet. The ground grid is toggled with `g`. Hotkeys: `SPACE` pause, `r` reverse, `+`/`-` speed, `.` step-frame, `[`/`]` Hz.

*Files: `robots/walking_robot.c`*

---

## 115. Perlin Terrain Bot — robots/perlin_terrain_bot.c

A self-balancing single-wheel robot navigating infinite 1D Perlin fBm terrain. The physics are an inverted pendulum (Lagrangian cart-pole) formulated on a sloped surface.

**Physics model:** The equation of motion is the Lagrangian cart-pole on slope:

```
θ_ddot = (g·sin(θ_eff) − ẍ·cos(θ_eff)) / L
```

where `θ_eff = θ − α` accounts for terrain slope `α`. The cart acceleration `ẍ` is the PID output.

**PID controller:** Proportional, integral, and derivative terms act on the pendulum angle error. A cascade position control loop generates a slope-corrected angle reference `θ_ref = −α×0.65`. Integral windup is clamped to prevent runaway accumulation on sustained slope.

**Terrain:** A 1D Perlin fBm ring buffer generates infinite scrolling terrain ahead of the bot. The slope `α` is computed as the finite-difference gradient of the height field at the bot's position.

**Interactive views (key `m` to cycle):**
- TELEMETRY — bar-graph display of P/I/D contributions and state variables
- EQUATIONS — live Lagrangian values substituted into the symbolic EOM
- PHASE SPACE — θ vs ω phase portrait with 240-sample convergence trail and regime classification (stable/settling/underdamped/overdamped)

**Gain presets (key `g` to cycle):** Six named presets (BALANCED, HIGH Kp, LOW Kp, NO Kd, HIGH Kd, NO Ki) are designed to teach PID tuning by showing characteristic failure modes side-by-side in the phase portrait.

*Files: `robots/perlin_terrain_bot.c`*

---

*This document describes the state of the framework as implemented across all C files in this repository. The canonical reference for any ambiguity is `physics/bounce_ball.c`.*
