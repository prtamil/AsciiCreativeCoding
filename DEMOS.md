# Demos

Per-program reference for every simulation in the repo. Each entry names the
algorithm, key data structures, and any non-obvious design choices. Programs
are grouped by topic folder; the folder location is given at the start of each
section. **337 programs across 19 top-level folders.**

For a high-level overview by folder, see the **Demos** section in
[README.md](README.md). For the underlying framework, see
[documentation/Framework.md](documentation/Framework.md); for the works each
program cites, see [documentation/Reference.md](documentation/Reference.md).

Build any single program with:

```bash
gcc -std=c11 -O2 -Wall -Wextra <folder>/<file>.c -o <name> -lncurses -lm
```

---

## Fluid Dynamics  (`fluid/`)

| Program | Algorithm |
|---------|-----------|
| `cfl_stability_explorer` | 2-D scalar wave equation via leapfrog finite differences; observes CFL stability bound (cfl ≤ 1.0) with eigenmode and Gaussian-drop initialisation |
| `complex_flowfield` | Lagrangian particle advection through four physics-generated velocity fields (curl-noise, vortex lattice, sine waves, radial spiral) via function-pointer dispatch |
| `flowfield` | Perlin noise → angle grid via `atan2(noise, noise)`; particles drift along the field; fractional Brownian motion for smooth evolution |
| `fluid_sph` | Smoothed Particle Hydrodynamics: 800–3000 particles with spatial-hash grid; density estimation + pressure/viscosity forces; symplectic Euler integration |
| `lattice_gas` | FHP-I cellular automaton: hexagonal grid, 6 bits per cell (one per direction); collision lookup table + streaming step; Navier-Stokes emerges from averaging |
| `navier_stokes` | Stam's stable fluids (1999): operator splitting (diffuse→project→advect→project); unconditionally stable; dye as passive scalar; EMA shade anti-flicker |
| `nuke` | Axisymmetric 2-D Stam fluids (temperature/density) with volumetric raymarching via Beer-Lambert; buoyancy-driven hot gas rising into a mushroom cloud |
| `reaction_diffusion` | Gray-Scott Turing patterns: forward Euler with 9-point isotropic Laplacian; seven (f, k) presets for spots/stripes/mazes/solitons |
| `reaction_wave` | FitzHugh-Nagumo excitable medium: cubic activator + slow inhibitor + diffusion; target rings, spirals, plane waves via impulse initial conditions |
| `shallow_water_solver` | Linearised 2-D SWE: height + (u,v) velocity fields; forward differences ∇h → momentum, backward ∇·v → continuity; 4 scenarios (dam break/radial/channel/obstacle) |
| `vorticity_streamfunction_solver` | 2-D Navier-Stokes via vorticity ω + streamfunction ψ (pressure-free); SOR Poisson solve for ψ; 4 scenarios (Kármán street, lid cavity, jet, step) with tracer particles |

## Physics  (`physics/`)

| Program | Algorithm |
|---------|-----------|
| `acoustic_wavesolver` | FDTD scalar pressure field — reflecting/absorbing boundaries, CFL stability, standing-wave room modes |
| `barnes_hut` | Barnes–Hut O(N log N) gravity — quadtree node aggregation, softened force, Keplerian orbits |
| `beam_bending` | Euler-Bernoulli FEM — modal superposition, 10 boundary/load combos, unconditionally stable time integration |
| `blackhole` | RK4 null-geodesic ray tracing — Schwarzschild metric, lensing lookup table, Doppler beaming |
| `bounce_ball` | Reference implementation — elastic bouncing in pixel-space physics, render interpolation, two coordinate spaces, cell aspect correction |
| `bubble_chamber` | Exact rotation matrix for Lorentz force — cyclotron spirals, ionisation drag, 5 particle species |
| `chain` | Position-Based Dynamics (PBD) with Verlet — iterative constraint projection, 10 presets, wind forcing |
| `charged_particles` | Naive O(N²) Coulomb force — bipolar 8-step charge palette, soft singularities, 4 patterns |
| `cloth` | Symplectic Euler spring-mass with 3 spring types — structural/shear/bend springs, 10 presets, 14 themes |
| `conjugate_gradient_linear_solver` | CG vs steepest descent on a 2-D energy bowl — iso-contour rendering, eigenmode panel, spectral view |
| `cymatics` | Analytic Chladni figures — Z(x,y) = cos(mπx)cos(nπy) − cos(nπx)cos(mπy), band thresholding |
| `elastic_collision` | Impulse-based hard-sphere collisions — conservation of momentum + energy, mass ∝ r², impulse formula |
| `gear` | Analytic polar gear + particle sparks — involute tooth approximation, 7-stage cooling pipeline |
| `gyroscope` | RK4 Euler equations + quaternion tracking — 8 presets (symmetric/asymmetric tops, gravity), 10 themes |
| `ising` | Metropolis Monte Carlo 2-D ferromagnetism — phase transition at Tc = 2.2692, domain walls, 10 patterns |
| `lattice_boltzman_fluid_simulator` | D2Q9 BGK lattice Boltzmann — Kármán vortex street, viscosity τ control, Reynolds tuning, 10 obstacles |
| `lorenz` | RK4 Lorenz attractor — 5 ε-offset ghost trajectories, Lyapunov chaos, depth cueing, lobe/density modes |
| `magnetic_field` | RK4 magnetic dipole field lines — monopole approximation, null points, 3-tier brightness ramp |
| `mass_spring_lattice` | Hooke's law + symplectic Euler — stress field rendering (rest = invisible), 4 strike scenarios |
| `membrane` | Explicit FD wave equation solver — 5-point Laplacian, 3 BC types, modal excitation, CFL monitoring |
| `multigrid_solver_visualizer` | V-cycle Poisson solver — Gauss-Seidel smoothing, grid hierarchy restrict/prolong, convergence tracking |
| `nbody` | Velocity Verlet softened gravity — 10 presets (Kepler to Galaxy Merger), O(N²) brute force |
| `pendulum` | RK4 N-link chaotic pendulum — Lagrangian mechanics, linear solver for accelerations, Lyapunov ghost |
| `pendulum_wave` | Analytic simple harmonic motion — integer-ratio frequencies, Berg–Marshall clap synchrony |
| `rigid_body` | Iterative impulse + Baumgarte stabilisation — AABB collisions, sleeping system, 10 themes |
| `rigid_multiple_bodies` | Same impulse solver with presets — 4 canned scenes (brick wall, beams, tower, pyramid), projectile firing |
| `rk_method_comparision` | Euler/RK2/RK4/Verlet side-by-side — phase portrait vs time series, stability limits, symplectic energy conservation |
| `schrodinger` | Crank-Nicolson TDSE + Thomas tridiagonal — complex wavefunction, phase-coloured bars, 4 potentials |
| `soft_body` | PBD mesh + point-in-polygon collision — Jordan-curve ray casting, Newton's 3rd law position split |
| `soft_multiple_bodies` | PBD + breakable distance constraints — kinematic pinning, fracture propagation, 4 destructible presets |
| `spring_pendulum` | Polar-coordinate spring pendulum — symplectic Euler, 2:1 parametric resonance, 10 modes |
| `stroke_engine` | Slider-crank kinematics + cycle state machine — 2/4/6-stroke thermodynamics, valve timing, 10 themes |
| `waves` | FDTD vs analytic closed-form superposition — source array, interference patterns, 5 presets (double-slit to ripple tank) |

## Procedural  (`procedural/`)

Six sub-folders: chaos, fields, fractals, generational, patterns, worldgen.

### Chaos  (`procedural/chaos/`)

| Program | Algorithm |
|---------|-----------|
| `bifurcation` | Logistic map, Feigenbaum period-doubling route to chaos; density rendering, cosine-gradient palette with log-tier mapping |
| `cobweb_diagram` | Logistic map f(x)=r·x(1−x) on unit square; cobweb (staircase) construction with age-banded segment ring buffer; 30 presets across the period-doubling cascade |
| `double_pendulum` | Lagrangian 4-D nonlinear ODE (θ₁,θ₂,ω₁,ω₂), RK4 integration; 30 presets across six behavioural classes; Bresenham rods, 4-band trail fade |
| `duffing_oscillator` | Driven nonlinear oscillator ẍ + γẋ − x + x³ = A·cos(ωt), RK4 with phase-space trail; 30 presets period-doubling to strange attractor |
| `henon_heiles` | 2-DOF Hamiltonian flow with cubic-saddle potential; Poincaré section at x=0; RK4 over an ensemble of orbits; 7 presets low-E regular → high-E chaotic sea with KAM islands |
| `magnetic_pendulum` | Damped pendulum above three magnets with softened 1/r² attraction; RK4 basin-of-attraction animation; 2 presets (weak/strong damping) showing fractal → Voronoi basins |
| `poincare_section` | Lorenz ODE (σ=10, ρ=28, β=8/3) with z-plane crossing detector; Poincaré section as a 2-D density histogram; 8-tier glyph ramp |
| `recurrence_plot` | Recurrence matrix R(i,j)=1 if \|xᵢ−xⱼ\|<ε, built progressively over N samples; 30 presets from trivial to Lorenz/Hénon/logistic chaos; texture classification |
| `rossler_attractor` | Rössler ODE with one nonlinear term, RK4; rotating 3-D camera (yaw/pitch); 16 presets fixed-point → chaos and exotic topologies (funnel/screw) |
| `sensitive_dependence` | 2–3 Lorenz trajectories seeded ε apart, parallel RK4 showing exponential divergence; 10 presets varying projection, perturbation size, and layout (overlay/split/log\|δ\|) |
| `standard_map` | Chirikov-Taylor area-preserving map on the 2-torus; 900-trajectory ensemble with density accumulation; 5 presets KAM tori → chaotic sea; 100-level intensity |
| `strange_attractor` | 10-preset zoo: 9 discrete 2-D chaotic maps (Hénon, Hopalong, Clifford, de Jong…) plus the Lorenz ODE; rotating 3-D projection with ghosts, depth-cued comet trail, starfield parallax |

### Fields  (`procedural/fields/`)

| Program | Algorithm |
|---------|-----------|
| `curl_noise_vector_field` | Divergence-free curl-of-noise v = (∂ψ/∂y, −∂ψ/∂x) with ψ a Perlin/fBm potential; 5 views (particles/vectors/potential/curl-magnitude/warped); central finite-difference gradient |
| `domain_warped_noise_iq_style` | Inigo Quilez domain warping f(x + g(x)) through 1–3 nesting levels plus a ridged variant; static fields drifting via time offset; 5 patterns, colour decoupled from intensity |
| `flow_field_particles` | 256 particles through 30 closed-form 2-D vector fields: rotational (vortices/spirals), radial (saddles/magnets), waves, nonlinear oscillators (Duffing/Van der Pol/Lotka); attractors mark singularities |
| `magnetic_fields` | Inverse-square monopole fields B(r)=Σ qᵢ(r−rᵢ)/\|r−rᵢ\|³; 30 configurations dipole → multipole grids/coils; iron-filings particle flow with N/S markers |
| `marching_squares_isocontours_showcase` | Marching squares: 4-bit per-cell classification, 16-case lookup, linear edge interpolation; 30 visualisations across single/multi/pair/topology/region/animated iso-contours |
| `midpoint_displacement_coastline` | 1-D midpoint-displacement fractal: recursive subdivision with halving jitter; Brownian output; 30 silhouette patterns (single line → multi-stack terraces); morphs between keyframes |
| `perin_noise_flow_showcase` | Perlin gradient noise (1985) with quintic fade; 30 patterns: flow, height, ridge/billow/zebra, domain warps, fBm stacks, texture composites (marble/wood/fire/clouds/caves) |
| `reaction_diffusion_gray_scott` | Gray-Scott PDE with 5-point Laplacian stencil; 30 (F,k) presets drawn from the Pearson phase diagram (spots/stripes/mazes to chaos) |
| `signed_distance_field_jfa_showcase` | Jump Flood Algorithm: log₂N passes of nearest-seed propagation → Euclidean distance field; 30 patterns from raw SDF to rings/Voronoi/contours/animated; wobbling seeds |
| `simplex_noise_clouds` | Simplex noise (Perlin 2001) on a triangular lattice, 3-vertex barycentric, isotropic; 30 patterns in 6 tiers (raw/mapped/turbulence/warped/composite/masked) |
| `value_noise_showcase` | Value noise: random scalars on a lattice + interpolation; 5 interpolation modes (smooth/linear/blocky/quintic/cosine); 30 patterns exposing axis-aligned grid artefacts |
| `vector_field_arrows_showcase` | 8-direction ASCII arrows via `atan2(vy,vx)`; 30 patterns: scalar gradients, analytic fields (radial/shear), physics (Coulomb/dipole), divergence-free flows, ODE portraits, animated |
| `worley_cellular_noise` | Worley/cellular noise: hashed feature points, 3×3 tile lookup for F1/F2/F3; 30 patterns from raw Fₙ to multi-metric (Manhattan/Chebyshev/anisotropic) and fBm stacks |

### Fractals  (`procedural/fractals/`)

| Program | Algorithm |
|---------|-----------|
| `apollonian` | Descartes Circle Theorem, recursive gap-filling, 7 depth levels, fill/outline toggle, hue-cycle animation |
| `barnsley` | IFS chaos game, 30 presets (ferns, trees, Sierpinski, dragons, curves), log-tone density mapping, 4-tier glyphs |
| `buddhabrot` | Escape-time Mandelbrot orbit accumulation, Buddha/Anti-Buddha mode, 5-tier log-tone density, random c sampling |
| `burning_ship` | Escape-time with absolute-value fold, 5 zoom presets, Fisher-Yates random reveal, 7 themes |
| `dragon_curve` | Paper-folding sequence generation (recursive rewriting), 13 generations max, turtle-traced segments, 7 colour bands |
| `fern` | IFS chaos game (single-point iteration), 4 fern variants, 80k iterations per cycle, colour by height, 8 themes |
| `julia` | Escape-time fixed-c Julia sets, 6 parameter presets, Fisher-Yates random reveal, 5 colour bands, 5 themes |
| `julia_explorer` | Split-screen Mandelbrot/Julia duality, interactive cursor + auto-wander, shared escape engine, cached left panel |
| `koch` | Edge-replacement snowflake subdivision, 5 levels (12→3072 segments), parametric curve→cell projection, 5 themes |
| `lightning` | Recursive tip branching (not DLA), 3 active tips, lean bias + fork probability, glow halo + shimmer, 5 themes |
| `l_system` | L-system string rewriting (Dragon, Hilbert, Sierpinski, Tree, Koch), turtle interpretation, auto-fit scaling |
| `lyapunov` | Logistic map with alternating parameter pair, Lyapunov-exponent classification, 6 symbol sequences, progressive scanline build |
| `mandelbrot` | Escape-time iteration z²+c, Fisher-Yates random reveal, 6 zoom presets, 10 themes |
| `newton_fractal` | Newton's method for z⁴−1, 4 root basins, convergence-speed banding, 5-theme palette, pan/zoom explorer |
| `sierpinski` | Recursive triangle subdivision (3^depth leaves), 7 depth levels, corner-colour triadic ramp, 10 themes |
| `snowflake` | Deterministic 6-fold dendritic branching, ±60° side branches, 6 depths, Bresenham line strokes |
| `tree_la` | Dielectric Breakdown Model (Laplace growth), Gauss-Seidel solver, 5 presets (Tree/Lightning/Coral/Frost/Discharge), η tunable 1–4 |

### Generational  (`procedural/generational/`)

| Program | Algorithm |
|---------|-----------|
| `ant_colony` | Ant Colony Optimisation — stigmergic pheromone trails; 12+ food patterns; 6 themes |
| `automaton_2d` | Larger-than-Life 2-D CA — neighbourhood radius R=1–10; 15 preset rules via summed-area table O(1) queries |
| `bsp_dungeon_showcase` | Binary Space Partition dungeon — recursive rectangle splitting; carve + corridor phases; BFS diameter solution |
| `cellular_automata_1d` | Wolfram elementary CA — 256 rules; 5 classes (fixed/periodic/chaotic/complex/fractal) |
| `cellular_automata_cave_4-5_rule_showcase` | CA cave (4-5 rule) — B5678/S45678 smoothing via double-buffer; animated wavefront sweep |
| `coral` | Radial DLA — particles drift inward, stick on contact; fractal branching D≈1.71; 15 presets |
| `delaunay_triangulation` | Bowyer-Watson incremental Delaunay — bad-triangle removal + recavitation; 15 presets |
| `diamond_square_heightmap_showcase` | Diamond-Square fractal heightmap — midpoint displacement + roughness decay; 6 terrain biomes; per-frame normalisation |
| `diffusion_map` | DLA vs Eden modes — fractal (D≈1.71) vs direct frontier (D→2); age-based colouring; 5 themes |
| `drunkards_walk_cave_showcase` | Drunkard's Walk cave — random walkers carve floors; multi-walker overlap forms caverns; respawn after MAX_AGE |
| `excitable` | Greenberg-Hastings excitable medium — N-state CA (N=5–20); refractory period tunes wave speed; 4 presets (spiral/double/rings/chaos) |
| `forest_fire` | Drossel–Schwabl forest-fire CA — 3-state (empty/tree/fire); self-organised criticality at critical p/f; 4 ecological presets |
| `hex_life` | Hexagonal Game of Life — B2/S34 on a 6-neighbour hex grid; offset rows; glow-based smooth birth/death |
| `langton` | Langton's Ant (Turmite) — 2-D Turing machine; rule string encodes turn per cell colour; 8 presets; highway emerges |
| `lenia` | Lenia (continuous Life) — convolution kernel + Gaussian growth; 3 self-organising creatures (Orbium/Aquarium/Scutium) |
| `life` | Conway's Game of Life + 5 rule variants — B/S bitmask rules; 6 presets (HighLife/Day&Night/Seeds/Morley/2×2); population histogram |
| `maze_backtracker` | Recursive-backtracker DFS maze — stack-based carving; 2× BFS diameter solver; 10 size presets |
| `maze` | Recursive-backtracker + BFS solver — 4-bit wall bitmask per cell; animated carve then solve; 3 sizes |
| `poission_disk_sampling_showcase` | Bridson fast Poisson-disk sampling — active list + background grid (r/√2 cells); K=30 attempts; blue-noise distribution |
| `sandpile` | Bak-Tang-Wiesenfeld Abelian sandpile — toppling threshold 4; self-organised criticality; 4-fold mandala symmetry; bloom/wave modes |
| `voronoi_region_map` | Brute-force nearest-seed Voronoi — O(W·H·N) compute; distance-order reveal animation; 8 seeds |
| `wator` | Wa-Tor predator-prey CA — fish breed/swim, sharks hunt/breed/starve; shuffled update; Lotka-Volterra oscillations; 4 regimes |
| `wfc_learn` | Wave Function Collapse (pedagogical) — 12-tile alphabet; entropy-heuristic collapse + constraint propagation; single-step mode |
| `wfc_showcase` | Wave Function Collapse (spectacle) — 34-tile ASCII pipes; 4-valued edge states; 5 seed ripples; 10 themes |
| `wilsons_algorithms_maze_showcase` | Wilson's algorithm — loop-erased random walks → uniform spanning tree; 2× BFS diameter solver; glow animations |

### Patterns  (`procedural/patterns/`)

| Program | Algorithm |
|---------|-----------|
| `maze_of_maze` | Recursive-backtracker DFS at two scales — outer maze contains an independent inner maze per cell; brightness-field animation; 4 patterns, 3 glyph sets |
| `penrose_pentagrid` | De Bruijn pentagrid dual method for P3 rhombuses; five families of parallel lines classify tiles by k-tuple parity; rotating viewport reveals aperiodicity |
| `penrose_tiling` | Penrose P3 deflation via Robinson triangles; acute/obtuse substitution scaled by golden ratio; animated subdivision with Bresenham raster |
| `quasicrystal` | Plane-wave interference at N directions; aperiodic long-range rotational order; 4 patterns by wave count (TRI/PENTA/HEPTA/UNDECA) × 4 glyph sets |
| `truchet_tiles` | Truchet tiling via per-cell rotation from a noise field; 4 distribution modes (random/fBm/bands/Voronoi) × 12 glyph sets = 48 combinations |
| `wang_tiles` | Wang edge-matching constraint solver; 16-tile complete set (2 colours/axis); placement biased by noise/sinusoid/swirl; 3 glyph styles (edges/blocks/wires) |

### Worldgen  (`procedural/worldgen/`)

| Program | Algorithm |
|---------|-----------|
| `cloud` | 15 fBm cloud presets (scale/octaves/warp/threshold); wind drift scrolls morphology (cumulus/cirrus/stratus/storm); pure-function per cell, no storage |
| `hydraulic` | Hydraulic erosion via Lagrangian droplets on an fBm heightmap; gradient descent + carrying capacity + bilinear erosion brush; dendritic networks self-organise; 15 view modes |
| `perlin_landscape` | Three-layer parallax fBm landscape (far/mid/near speeds); octave stacking with gain/lacunarity; deterministic star field; painter's-algorithm compositing |
| `procedural_city` | L-system BSP of rectangle blocks; stochastic H/V split biased by aspect ratio; per-lot zoning gradient; build animation follows derivation depth; window-light twinkle |
| `procedural_constellation` | Poisson-jittered anchor stars; 15 graph topologies (MST/chain/loop/spoke/wheel/fan/spiral…); Bresenham line-trace animation; procedural Latin-flavoured naming |
| `procedural_galaxy` | Logarithmic spiral arms modulated by bulge/disk/proximity density; hash-gate star placement; 15 morphologies (spiral/barred/elliptical/starburst…); optional fBm arm roughening |
| `procedural_star_field_parallax_noise_showcase` | Hash-based infinite parallax star field across 4 depth layers; 15 modes (starfield/twinkle/nebula/warp/tunnel/wormhole/pulsar/supernova…); fBm nebula backdrop |
| `tectonic` | Plate tectonics — Voronoi plate assignment, boundary classification (convergent/divergent/transform), elevation via distance-to-boundary + fBm, 8 biome bins; 15 cartographic views |
| `terrain` | Diamond-square midpoint displacement (65×65 grid); thermal weathering (talus threshold, slope transport); height-to-contour rasterisation with bilinear interpolation |

## Grid Systems  (`grids/`)

All major grid families are split into two parallel sub-folders: a **display**
folder showing the bare grid, and a **placement** folder where a cursor can
deposit objects on that grid. Four families: rectangular (14 displays + 4
unified placement editors), polar (7 + 4), hex (7 + 4), and triangular (12 + 24).
See [grids/README.md](grids/README.md) for the reading order.

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

## Rasterizer  (`raster/`)

| Program | Algorithm |
|---------|-----------|
| `bloom_finale` | Deferred shading + SSAO + bloom; G-buffer (pos/normal/albedo/emissive), per-pixel hemisphere AO, separable Gaussian blur on bright pixels |
| `cube_raster` | Forward rasterisation (MVP→divide→screen→cull→barycentric→z-test); flat-normal cube; 4 shader modes (phong/toon/normals/wireframe); 6×6×6 RGB cube + Bourke ramp |
| `deferred_rendering_pipeline` | Two-pass deferred: geometry pass writes G-buffer (pos/normal/albedo); light pass accumulates per-light Blinn-Phong; up to 8 point lights |
| `displace_raster` | Vertex displacement along surface normal via scalar field; central-difference normal recompute; 4 field modes (ripple/wave/pulse/spiky); barycentric raster |
| `donut` | Analytic torus point-sampling; (θ,φ) walk with Lambertian shading; perspective projection with K1/K2 scaling; 6 themes; z-buffer + glyph store — the original spinning donut |
| `mandelbulb_raster` | Hybrid raymarch-then-rasterise: UV-sphere ray-march from outside inward to build a mesh, then forward rasteriser; power 2–16 |
| `marching_cubes` | Marching cubes isosurface extraction from a metaball scalar field every frame; 256-case lookup tables (EDGE_TABLE/TRI_TABLE); real-time topology changes; 7 themes |
| `neon_edges` | Screen-space edge detection via Sobel on G-buffer depth + normal channels; flagged edges drive HDR neon colour into the bloom pipeline |
| `shadow_mapping` | Two-pass shadow mapping (Williams 1978): light-view depth pass → shadow map, camera pass does depth compare + PCF; orthographic light view |
| `sphere_raster` | Forward rasterisation of a UV-tessellated sphere; smooth per-vertex normals; 4 shaders (phong/toon/normals/wireframe); barycentric interpolation |
| `ssao_pipeline` | Screen-space ambient occlusion: K hemisphere samples per pixel projected to screen + depth lookup; 3×3 blur; modulates ambient only |
| `sun_solar` | 2-D screen-space distance-to-centre classifier; Eddington limb darkening, fBm convection granulation, sunspots; exponential corona; parabolic flares; 5 themes |
| `torus_raster` | Forward rasterisation of a UV-tessellated torus; closed-form normals (subtract ring centre); 4 shaders (phong/toon/normals/wireframe) |
| `wireframe` | Wireframe rendering: vertex/edge tables → Bresenham line drawing; perspective projection with aspect correction; 4 shapes (cube/sphere/pyramid/torus) |

## Raymarcher  (`raymarcher/`)

| Program | Algorithm |
|---------|-----------|
| `csg_atlas` | 13 CSG boolean operators (hard/smooth/round/chamfer × union/intersect/subtract/xor); animated operands; 4 debug overlays |
| `kifs_fractal` | Kaleidoscopic IFS fractals: fold-then-contract iteration; 3 presets (Sierpinski tetrahedron/Menger sponge/rotating crystal); orbit-trap colouring |
| `mandelbulb` | 3-D Mandelbulb (power 8 canonical); distance estimator with derivative tracking; smooth-iteration colouring; soft shadows + AO via march step count |
| `metaballs` | Six orbiting spheres blended with polynomial smooth-min; curvature-based colour bands; 2×2 SSAA option; soft shadows via Quílez penumbra |
| `raymarcher` | Textbook sphere trace (Hart 1996): single sphere SDF; 4 debug overlays; closed-form normal; Phong shading |
| `raymarcher_cube` | Sphere-traced box SDF; tetrahedral 4-tap normal; same March loop as `raymarcher.c`; rotated via point transformation |
| `raymarcher_primitives` | 17 SDF primitive catalogue (sphere/box/torus/cone/capsule/octahedron…); function-pointer dispatch; Floyd-Steinberg dithering; 92-char Bourke ramp |
| `sdf_gallery` | 5 scenes of SDF composition: blend/boolean/twist/repeat/sculpt; 3 lighting modes (N·V/Phong/Flat); 4 debug overlays; per-frame trig cache |

## Analytic Ray Tracing  (`raytracing/`)

| Program | Algorithm |
|---------|-----------|
| `capsule_raytrace` | Ray-capsule: decomposed into cylinder body + 2 spherical caps; quadratic solve per part; 4 shading modes; 3 debug overlays; 20 PBR materials |
| `cube_raytrace` | Slab method: ray vs 3 axis-aligned slabs, interval overlap; 4 shading modes; wireframe via face-edge distance; 20 PBR materials |
| `forest_god_rays` | Volumetric path tracer: march through height-decaying mist; NEE asks if sun is visible from each sample (tree occlusion); Henyey-Greenstein phase; 4 Kelvin presets |
| `god_rays_window` | Volumetric path tracer: uniform interior mist; slit-aware wall test (pointed-arch openings); 10 windows per 2 rows; NEE + phase function |
| `path_tracer` | Progressive Monte Carlo path tracer: 7 passes (camera ray → intersect → shade → bounce → throughput → terminate → accumulate); Cornell Box; 4 debug overlays |
| `saturn_with_rings` | Ray-sphere + ray-ring; limb darkening + atmospheric rim; soft ring shadows + forward-scatter; Cassini gap; 4 patterns (Saturn/Uranus/Ringed-Earth/Exoplanet) |
| `solar_eclipse` | Volume-rendered eclipse: photosphere + corona (∝r⁻³) + chromosphere + prominences; lunar terrain with per-seed Bailey's-bead valleys; penumbra via disc overlap; eye adaptation |
| `sphere_raytrace` | Quadratic ray-sphere intersection; 3-point lighting (key/fill/rim); 4 shading modes (phong/normals/fresnel/depth); 20 PBR materials |
| `torus_raytrace` | Quartic ray-torus: Horner-form polynomial + scan-and-bisect solver; closest-point normal formula; 4 shading modes; 20 PBR materials |
| `tunnel` | Ray-cylinder (closed-form quadratic); cylindrical UV mapping; 4 patterns (rings/checker/spokes/grid); distance fog; sway + roll camera; 6 themes |

## Artistic  (`artistic/`)

| Program | Algorithm |
|---------|-----------|
| `aurora` | Aurora borealis — closed-form side-view scene (no state grid), per-cell function of (col,row,time,seed); 18-colour ramp with colour-wave, drapery folds, shimmer, stateless stars; 4 themes |
| `bat` | V-formation particle system — 3 formations burst outward; triangular Pascal rows; 4-frame wing cycle; 6 preset headings; dynamic `+/-` rows (1–6) |
| `bonsai` | Two-pass procedural tree — stochastic recursive skeleton (5 styles: chokkan/moyogi/shakan/kengai/bunjin); Bresenham raster + aspect-corrected ellipse foliage; per-frame wind rustle |
| `dna` | 10 DNA/RNA structures — parametric helix rendering (B/A/Z-DNA, triple helix, G-quadruplex, hairpin, replication fork, cruciform, plasmid, ladder); depth-cued backbone sinusoids; 6 themes |
| `dune_rocket` | Homing missiles — proportional-navigation steering; ballistic acceleration; trail ring buffer; explosion sparks with gravity; terrain heightmap + scorch decay; 28 rockets max |
| `dune_sandworm` | Segmented worm — chain-of-circles follower (constraint relaxation), 50 segments; underground sinusoidal path + parabolic breach arc; sand ripples + spray; direction-dependent glyphs |
| `fire_tornado` | Three layered systems — rotating ember pool (cylindrical particles), 1-D base heat mat with diffusion/injection, spark pool with gravity; wind sway; 5-stop heat ramp |
| `galaxy` | Spiral galaxy — 3000 stars on 2–4 logarithmic arms; flat rotation curve; differential rotation (ω∝1/r) winds arms over time; brightness-decay trails; density→glyph ramp; 3 zones |
| `hindu_mandalas` | 30 parametric Hindu mandalas — 6 radial primitives (circle/dots/petals/polygon/star/rays) layered as rings; 20 simple + 10 complex forms; progressive build; 4 themes |
| `hurricane` | Rankine vortex top-view — cloud particles orbit in polar coords; solid-body rotation inside eye, 1/r decay outside; radial inflow recycling; 15 presets; 3 radial zones |
| `islamic_mandalas` | 30 parametric Islamic patterns — 6 primitives (circle/polygon/star-polygon/star-shape/interlock/rays); 20 simple + 10 complex; progressive reveal; 4 themes (Iznik/Persian/Andalusian/Mamluk) |
| `jellyfish` | Physics pulse locomotion — FSM (IDLE/CONTRACT/GLIDE/EXPAND); parametric half-ellipse bell with time-varying radii; tapered tentacle-chain followers; jet thrust + gravity + drag |
| `lava_lamp` | Metaball scalar field — N blobs with temperature T; field f=Σ rᵢ²/dᵢ²; threshold-fill interior; buoyancy physics; blob merging/splitting; heat ramp; 3–10 blobs |
| `leaf_fall` | Two-phase — stochastic recursive tree (branching DFS) seeded into grid, then matrix-rain leaf fall with per-column streamers (white head, green trail); 4096-leaf pool with stagger |
| `led_number_morph` | Spring-mass particle morphing — 336 particles form 7-segment digits; per-particle spring-to-target with critical damping; active/inactive → spread/centre drift; orientation-aware glyphs; 5 themes |
| `nebula` | Multi-octave fBm gas field — value noise with 2 parallax layers; star catalogue with twinkle; shock-wave birth events (radial expansion, age-fade); 50–400 stars |
| `particle_number_morph` | Greedy nearest-neighbour matching + LERP — 9×7 bitmap font expanded to 500 particles; on digit change snapshot/match/lerp with smoothstep (no springs); idle fade to centre; 5 themes |
| `phoenix` | Two particle systems + FSM — body particles snap to an anchor template (6-phase: PERCH/IGNITE/BLAZE/COLLAPSE/ASH/REBIRTH); free-flight fire physics (buoyancy/shear/drag/cool); spark pool; 4 themes |
| `plasma` | Analytic plasma — sum of 4 sinusoids at varying spatial/temporal frequencies; aspect-corrected radial term; phase rotates at CYCLE_HZ; palette cycling; 4 frequency presets |
| `railwaymap` | Procedural transit map — 5 path templates routed on an 8×6 logical grid; canvas cell stores h/v line → junction detection; 12–15 animated trains; perpendicular station naming; 10 themes |
| `sand_art` | Falling-sand CA with momentum — per-cell vertical velocity; quarter-ellipse curved hourglass cavity; diagonal fall when blocked; auto-flip + time tracking; 4 patterns × 6 themes |
| `volcano` | Five-layer composition — sky gradient, fBm mountain silhouette, lava flows, fBm plume column, 1024-particle pool (bombs/embers/ash/sparks); 4 eruption patterns; 6 themes |
| `xrayswarm` | Ray swarm — queen wanders (Brownian); 20 workers per swarm DIVERGE outward, pause, CONVERGE back (homing); 48-slot brightness-fading rays; 3-pass render; 1–5 swarms |

## Animation & Kinematics  (`animation/`)

| Program | Algorithm |
|---------|-----------|
| `fk_centipede` | Path-following FK body (trail buffer) + sinusoidal stateless FK legs/antennae; contralateral antiphase gait with travelling-wave phase offsets; Reynolds boundary repulsion |
| `fk_helloworld` | Forward kinematics — given joint angles θ1, θ2, compose rotations to find positions; deterministic hand position; no iterations |
| `fk_ik_helloworld` | Toggle FK (forward propagation) vs 3-link hybrid IK (2-link analytical + L3 wrist aim); the 2+1 split mirrors real animation rigs |
| `fk_tentacle_forest` | Stateless FK for fixed-root chains; per-segment cumulative-angle bend with per-strand phase offset + frequency detuning to break lockstep; no trail buffer |
| `hexpod_tripod` | Interlocking tripod gait (two tripods alternate planted/swinging); 2-joint analytical IK (law of cosines) per leg; knee-side sign flip for left/right splay |
| `ik_arm_reach` | FABRIK iterative IK (forward/backward passes); Lissajous figure-8 autonomous target (1:2 ratio); reachability check with reach-horizon visualisation |
| `ik_helloworld` | Analytical 2-link IK via law of cosines; one sqrt + one acos + one atan2; two valid solutions (elbow up/down flip) |
| `ik_scorpin` | Hybrid — trail-buffer body FK + 2-joint analytical IK legs + per-leg autonomous gait + stateless FK tail with cumulative base curl + sin sway |
| `ik_spider` | Three systems — path-following FK body (trail buffer), 2-joint IK legs (reachable-annulus clamp), per-leg autonomous gait (max N_LEGS/2 airborne) |
| `ik_tentacle_seek` | FABRIK with per-joint angle constraints (~63° max bend); backward/forward passes with clamping; Lissajous 1:1.7 target + low-pass velocity smoothing |
| `ragdoll_figure` | Position-Verlet + distance-constraint projection (Jakobsen, 8 passes); gravity + wind; slanted-platform collisions via position clamp + reflection |
| `ragdoll_ropes` | Verlet particles + iterative distance constraints; sinusoidal lateral wind with per-rope phase offset (Mexican-wave sway); 6 constraint passes per tick |
| `snake_forward_kinematics` | Path-following FK (trail-buffer arc-length sampler); sinusoidal auto-steering + Reynolds-style edge-bias soft fence (quadratic falloff) |
| `snake_inverse_kinematics` | IK goal-seeking head (atan2 + step toward wander target) + path-following FK body; wander target = sum of 3 incommensurable sines + billiard-ball edge bounce |

## Robots  (`robots/`)

| Program | Algorithm |
|---------|-----------|
| `diff_drive_robot` | Two-step kinematics — inverse: map (v_cmd, ω_cmd) to wheel speeds with accel ramps; forward: integrate body speed v=(vL+vR)/2 and spin ω=(vR−vL)/axle into pose; nonholonomic by construction |
| `moving_jump_spring_leg_robot` | Three-phase FSM — COMPRESS (spring loads quadratically), RELEASE (energy → upward velocity), FLIGHT (parabolic arc under gravity); terrain-adaptive launch angle; value-noise terrain |
| `perlin_terrain_bot` | Self-balancing wheel bot — 5-octave fBm terrain + inverted pendulum + PID controller (with slope feed-forward); 4× sub-stepping keeps stiff dynamics stable |
| `walking_robot` | Procedural sinusoidal gait — SWING uses FK (thigh/knee phase functions), STANCE uses 2-link IK with locked foot contact; alternating legs at π offset |

## Flocking & Emergence  (`flocking/`)

| Program | Algorithm |
|---------|-----------|
| `crowd` | Reynolds steering forces (seek/flee/separate/align/cohere) with six switchable behaviours (wander/flock/panic/gather/follow/queue) |
| `flocking` | Five flocking modes across three leader-led flocks: boids, chase, Vicsek (align+noise), orbit formation, predator-prey |
| `murmuration` | 800 boids + spatial-hash O(N·k) force accumulation; density-field rendering (glyph ramp `.,:;oO*#@`) instead of per-bird; hawk DIVE mechanic |
| `shepherd` | Strömbom border-collie algorithm — dog positions itself outside outlier sheep relative to pen centre; 5 compound shapes (circle→square→triangle→hexagon→octagon) |
| `slime_mold` | Jeff Jones (2010) Physarum model — 3-sensor sense→rotate→move→deposit loop; grid diffuse/decay; self-organising Steiner-tree networks |
| `swarm_gen_numbers` | Reynolds swarm with greedy slot assignment — 120 agents form digits 0–9 via 3×3 slot subdivision; 10 strategies (drift/rush/flow/orbit/flock/pulse/vortex/gravity/spring/wave) |
| `war` | Two-faction melee+archer battle — 4-state FSM (ADVANCE/COMBAT/FLEE/DEAD); real projectile arrows; 6 strategy presets |

## Algorithms  (`algorithms/`)

| Program | Algorithm |
|---------|-----------|
| `bsp_tree` | Binary Space Partition tree — axis-alternating midpoint splits, O(log N) balance; O(√N + k) range queries via AABB pruning; ancestor of Doom/Quake rendering |
| `convex_hull` | Graham scan (polar sort + stack sweep, O(N log N)) and Jarvis march (O(N·h)) raced side-by-side; cross product determines turn direction |
| `graph_search` | BFS (queue), DFS (stack), A* (priority queue + Euclidean heuristic) on a Fruchterman-Reingold force-directed graph layout |
| `kd_tree` | K-D tree with alternating-axis splits; single point per node as split plane; O(log N) insert, O(√N + k) range query; bounding-box pruning |
| `marching_squares` | Marching squares — 4-bit corner classification + 16-case edge-crossing table + interpolation; metaball potential field with aspect correction |
| `network_sim` | SIR epidemic on a Watts-Strogatz small-world network (K=4 ring, 15% rewiring); R0 threshold; network ring + stacked epidemic curve |
| `quad_tree_helloworld` | Quadtree fundamentals — INSERT/SUBDIVIDE (NW/NE/SW/SE)/QUERY with AABB pruning; pool allocator (no malloc); phase-machine demo |
| `quadtree` | Quadtree library — leaf ≤4 points, subdivides into four equal children; AABB pruning → O(log N + k) range queries; half-open intervals avoid midpoint ambiguity |
| `sort_vis` | Five coroutine-style sorts (one op per frame): Bubble, Insertion, Selection, Quicksort (Lomuto), Heapsort; colour codes compare/swap/sorted |
| `visibility_polygon` | Angular-sweep visibility polygon — cast 3 rays per endpoint (θ−ε, θ, θ+ε), sort hits by angle, join; ray-segment intersection via cross product; aspect-corrected |
| `voronoi` | Brute-force Voronoi (O(cells × seeds)/frame); Langevin (Ornstein-Uhlenbeck) seed drift; border detection via distance gap; small N ≤ 30 |

## Geometry  (`geometry/`)

| Program | Algorithm |
|---------|-----------|
| `delaunay_triangulation` | Bowyer-Watson incremental insertion — find bad triangles, trace boundary hole, refill with the new point; empty-circumcircle property; O(N log N) |
| `lissajous` | Damped Lissajous figures — x=sin(fx·t+φ)·e^(−λt), y=sin(fy·t)·e^(−λt); rational ratios close; phase drifts; 4 brightness levels; 8 named ratio shapes |
| `maurer_rose` | Discrete-sample rose r=cos(n·θ) at fixed d-degree intervals joined by chords; density buffer with sqrt-scaled brightness tiers; theme palettes; parameter drift near integer presets |
| `spirograph` | Hypotrochoid parametric curves; float canvas with per-tick fade; three simultaneous curves with slow r drift |

## Signal Processing  (`signal/`)

| Program | Algorithm |
|---------|-----------|
| `aliasing` | Nyquist/sampling-theorem demo — true frequency sweeps past fs/2, sampled version folds back; fold-back arithmetic; continuous + sampled panels with Bresenham reconstruction |
| `convolution_helloworld` | 1-D convolution slide-multiply-sum; six kernels (Identity/Box/Gaussian/Edge/Sharpen/Inverse); five inputs; live multiply-and-sum overlay |
| `dft_helloworld` | Naive O(N²) DFT — double-loop projection onto complex exponentials; spectrum bar chart with auto-sweep or manual frequency; phase + bin-info debug panels |
| `epicycles` | DFT on 2-D closed curves — coefficients become rotating arms; 20 parametric shapes; sort by amplitude → animate chain; auto-grow toggle |
| `fft_helloworld` | Cooley-Tukey radix-2 FFT — bit-reversal permutation + log₂N butterfly stages; every frame verifies FFT against the DFT reference (error < 1e-4) |
| `fft_vis` | FFT visualiser — three panels (waveform / magnitude spectrum / spectrogram waterfall); seven signals; four windows (Rectangular/Hann/Hamming/Blackman) |
| `fir_filter` | FIR design by recipe — ideal sinc → truncate to K=21 taps → spectral inversion/difference → window taper; low/high/band-pass; linear phase response |
| `fourier_draw` | Interactive Fourier drawing — trace a path with arrow keys, ENTER → DFT → epicycle chain redraws it; arc-length resampling; theme palettes |
| `fourier_shapes` | Epicycle chain over 21 preset parametric shapes; Parseval energy bar (cumulative power captured by M arms); tier labels by convergence speed |
| `idft_helloworld` | Inverse DFT — five modes: round-trip verify, low-pass/high-pass via zeroed bins, magnitude-only, phase-only; shows that phase carries shape |

## Particle Systems  (`particle_systems/`)

| Program | Algorithm |
|---------|-----------|
| `aafire_port` | aalib cellular automaton — 5-neighbour stencil; gamma + Floyd-Steinberg dithering + perceptual LUT to 9 glyphs; 10 themes + 4 debug overlays |
| `burst` | Radial bursts with 3-state FSM (IDLE→FLASH→LIVE); wave-staggered fan emission; exponential drag + life-keyed ramp; persistent scorch grid; 10 themes |
| `comet` | Moving-emitter trails — particles spawn with no inherited velocity; Bresenham fractional emission; impact detonation with radial fan; 3 patterns; data-driven engine |
| `constellation` | Proximity graph (O(N²) pair scan/frame); bounded random walk (Ornstein-Uhlenbeck); Bresenham line render; pixel/cell split physics; render lerp; 7 themes |
| `curl_noise_particles` | Particles advected by 2-D curl-noise velocity (∂N/∂y, −∂N/∂x); divergence-free Perlin fBm + finite-difference curl; 4 patterns (CALM/TURBULENT/HURRICANE/WIND_TUNNEL) |
| `embers` | Heat-rising particles with constant buoyancy; temperature-driven colour ramp; per-tick turbulence in vx; 4 patterns (BONFIRE/FORGE/DRAGON/HEARTH); 10 themes |
| `fire` | Three switchable algorithms — Doom-CA, Lagrangian particle splat, plasma sine-sum; shared heat grid + gamma Floyd-Steinberg + 9-glyph LUT; 6 tints; wind + fuel control |
| `fireworks` | Two-phase rocket + burst — LAUNCH (parabolic arc with drag) → BURST (radial sparks, Gaussian speed); explicit Euler; 7-colour palette per rocket; fixed pool |
| `fountain` | Pool-based two-species (drops + splashes); cone ejection with jitter; constant gravity; 4 patterns (GEYSER/FOUNTAIN/WATERFALL/VOLCANIC); height-based ramp |
| `kaboom` | Two-layer 3-D explosion — wave layer (scalar field with petal lobes) + blob layer (800 unit-sphere dirs via pinhole camera); closed-form position(t); depth-bucketed glyph; 6 ramps |
| `particle_engine` | Symplectic Euler + modular force accumulation (gravity/drag); 3 boundary policies (BOUNCE/KILL/WRAP); 4 render modes; preset library (fountain/fireworks/rain/explosion) |
| `pixel_dissolve` | Bitmap-font particles (5×7 glyphs, one per "on" pixel); 3-phase (ASSEMBLE→HOLD→DISSOLVE); damped harmonic spring; 4 dissolve patterns; horizontal colour gradient |
| `rain` | Pool-based drops + splashes; motion-blur streak with slope-picked glyph; gravity; splash arcs on impact; 4 patterns (DRIZZLE/SHOWER/STORM/MONSOON); wind control |
| `sand` | Probabilistic CA (bottom-to-top); two-diagonal slide (~45° repose); wind-drift for unsettled grains; per-grain age encodes compaction; 10 themes |
| `smoke` | Five physics algorithms — particle puffs, vortex advection, curl-noise advection, buoyancy plume, breeze; semi-Lagrangian backtrace for 1–4; Floyd-Steinberg + 9-glyph LUT; 10 themes |
| `snow` | Pool-based flakes + 1-D pile-height array; sin-sway per flake; contact detection + valley-fill deposit (neighbours roll down); 3 patterns; 8-step glyph ramp |
| `sparks` | High-speed particles with elastic floor bounce; motion-blur trail history; 10 patterns incl. SPARKLER (mid-flight splits) and SPINNER (rotating wheel); heat-ramp themes |
| `vortex` | Polar-coordinate particles (r, θ); inflow + angular momentum (1/r); log-spiral + constant-rotation modes; 10 patterns (WHIRLPOOL/TORNADO/BLACK_HOLE/GALAXY…); tangent trail render |

## Matrix Rain  (`matrix_rain/`)

| Program | Algorithm |
|---------|-----------|
| `fireworks_rain` | Rockets (IDLE→RISING→EXPLODED FSM) burst into 72 sparks; each spark trails its last 16 head positions in a ring buffer; glyph reroll every frame for shimmer |
| `matrix_rain` | Per-column streams with float head position advancing by speed·dt; 8-slot glyph cache rerolls each tick; 5-band fade (HOT→BRIGHT→MID→DARK→FADE) |
| `matrix_snowflake` | Rain streams freeze into a per-column snow pile when the head reaches its top; varied column speeds; pile resets when full; FALL↔FLASH state machine |
| `pulsar_rain` | N rotating beams (1–16) sweep from an `@` core with an angular wake of rerolled glyphs; radial samples walk outward; dim-first painter prevents overwrites |
| `sun_rain` | 180 radial rays shoot from an `@` core at random speeds with stagger offsets; trail fades (HOT→5 shades→FADE); rays recycle at the horizon |

## AI  (`Ai/`)

| Program | Algorithm |
|---------|-----------|
| `genetic_rocket` | Holland GA (1975) — rockets carry a fixed-lifespan genome of 2-D force vectors; fitness = 1/(distance+1) with hit/crash bonuses; fitness-proportional mating + single-point crossover + per-gene mutation |
| `neural_net_vis` | Feed-forward network layout — layer i at an x-fraction, neuron j at a y-fraction, full connectivity; particles hop edge-by-edge; linear interpolation for line draw; O(L·N²) connections |

## Turtle Graphics  (`turtle/`)

| Program | Algorithm |
|---------|-----------|
| `duo_poly` | Regular n-gon via turtle — vertices at θ_k = θ₀ + k·(2π/n); DDA line raster; aspect fix (ASPECT=0.5); two turtles cycle 3–12 sides with a 2-second pause |

## ncurses Basics  (`ncurses_basics/`)

| Program | Algorithm |
|---------|-----------|
| `aspect_ratio` | Parametric circle x=cx+r·cos θ, y=cy+r·sin θ; X scaled by ASPECT (≈2) to compensate for the 2:1 cell height-to-width ratio |
| `framework` | Reference template — fixed-step accumulator (sim_accum drains in SIM_TICK_NS steps); alpha-lerp render interpolation; ncurses double-buffer with doupdate diff |
| `test_framework` | Framework variant with a key generator cycling random ASCII characters; canonical 8-section structure |
| `tst_lines_cols` | Minimal demo — reads ncurses globals LINES and COLS via `initscr()`; updates on KEY_RESIZE |
