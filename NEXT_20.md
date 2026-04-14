# Next 20 Programs

Ideas ranked by learning value and visual payoff. Each builds on existing project knowledge.
All programs follow the §1–§9 framework: config → clock → color → vec3/coords → math → canvas → scene → screen → app.
Physics in pixel space (CELL_W=8, CELL_H=16). Fixed timestep accumulator. Alpha render interpolation.
Sleep before render. erase → scene_draw → HUD → wnoutrefresh → doupdate.

---

## Tier 1 — Fluid Solvers

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 1 | `lbm.c` | fluid/ | Lattice Boltzmann D2Q9 — industry-standard grid fluid: density, velocity, vorticity fields with obstacle drag; presets: Kármán vortex street, lid-driven cavity, Poiseuille flow, double slit | D2Q9: 9 distribution functions f_i per cell; BGK collision `f_i' = f_i − (1/τ)(f_i − f_i^eq)`; equilibrium `f_i^eq = ρ w_i [1 + (c_i·u)/cs² + (c_i·u)²/2cs⁴ − u²/2cs²]`; streaming shifts f_i along c_i; ρ=Σf_i, u=Σf_i·c_i/ρ; vorticity `ω = ∂uy/∂x − ∂ux/∂y` → signed color ramp; grid maps directly to terminal cells — no px_to_cell needed; obstacle cells use bounce-back (reverse direction); τ controls viscosity via `ν = cs²(τ−0.5)dt`; 5 themes |
| 6 | `fluid_sph.c` | fluid/ | Smoothed Particle Hydrodynamics — 2D particle fluid with pressure, viscosity, surface tension; splash simulation; dam break preset | N=800 particles in pixel space (CELL_W=8/CELL_H=16); poly6 density kernel `W(r,h)=(315/64πh⁹)(h²−r²)³`; spiky pressure gradient `∇W`; viscosity Laplacian `∇²W`; density `ρ_i=Σ m_j W(r_ij,h)`; pressure `P_i=k(ρ_i−ρ_0)`; spatial hash grid for O(1) neighbour search; symplectic Euler integration; surface tension via density threshold; density → character density ramp; velocity magnitude → color pair; 5 presets; 5 themes |

---

## Tier 2 — Physics Upgrades

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 2 | `fem_solid.c` | physics/ | Finite Element Method soft solid — 2D triangle mesh elasticity; beam bending, rubber plate compression, elastic cantilever; von Mises stress heatmap | Triangle mesh over terminal grid; per-element stiffness matrix `K_e = ∫ B^T D B dA`; global assembly `K_global u = f`; linear solve via Gauss elimination or Conjugate Gradient; displacement field → stress → von Mises `σ_v = √(σ_x²−σ_xσ_y+σ_y²+3τ²)`; stress magnitude → color pair + char density; fixed boundary nodes (Dirichlet BC); px_to_cell in draw only; 5 themes |
| 5 | `rigid_body_rot.c` | physics/ | 2D rigid body with full angular momentum — torque, moment of inertia, spin integration; toppling towers, spinning cubes, rolling friction, angular collisions | Extends rigid_body.c: add `float angle, omega` to Body struct; moment of inertia `I = (1/12)m(w²+h²)` box, `I = (1/2)mr²` disk; contact point `r = contact − centroid`; rotational impulse denominator `1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b`; torque from friction `τ = r × F_friction`; spin `ω += τ/I · dt`; draw rotated AABB via Bresenham 4-corner lines; px_to_cell for corner draws; 5 themes |

---

## Tier 3 — Algorithmic Simulation

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 3 | `barnes_hut.c` | physics/ | **DONE** Barnes–Hut O(n log n) gravity — 800-body galaxy with quadtree acceleration; flat rotation curve disk; Keplerian orbits; brightness accumulator glow; speed-normalized coloring | Quadtree node pool (no malloc); opening angle criterion `s/d < θ=0.5`; incremental centre-of-mass; central BH anchor body (skipped in integration); brightness accumulator DECAY=0.93; rolling v_max for adaptive colormap; 3 presets; 5 themes |
| 4 | `maxwell_fdtd.c` | physics/ | 2D Maxwell FDTD electromagnetic solver — TM mode (Ez, Hx, Hy); waveguide, diffraction slit, antenna emission, dielectric refraction; PML absorbing boundary | Yee grid: Ez at cell centres, Hx/Hy at edges; update equations: `Ez[i][j] += dt/ε·(ΔHy/Δx − ΔHx/Δy)`, H-field updates; CFL stability `dt ≤ dx/(c√2)`; PML 8-cell boundary with conductivity ramp `σ(x)=σ_max·(x/d)²`; point source sinusoidal injection; ε_r per cell for dielectric slab; Ez → signed 8-level color ramp; terminal grid IS the Yee grid; 4 presets; 5 themes |

---

## Tier 4 — Raymarcher (distance fields)

Raymarching shines for smooth implicit surfaces, fractal geometry, boolean composition, and cinematic glow + AO + soft shadow effects. Every pixel is one ray; the terminal grid is the virtual framebuffer.

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 7 | `mandelbulb_explorer.c` | raymarcher/ | **DONE** 3D Mandelbulb fractal — orbit trap colouring, soft shadow, AO, camera orbit, power parameter, zoom animation, power morphing, edge glow, starfield, animated palette | Spherical power formula `z^p → r^p·(sin(pθ)cos(pφ), sin(pθ)sin(pφ), cos(pθ))`; DE `0.5·log(r)·r/dr`; smooth coloring `mu = iter+1 - log(log(r)/log(bail))/log(p)`; near-miss glow via min_d tracking; rim lighting; color phase rotation; progressive ROWS_PER_TICK rendering; 7 themes |
| 8 | `sdf_gallery.c` | raymarcher/ | SDF Sculpture Gallery — boolean composition of primitives: sphere, capsule, torus, cone, rounded box; smooth union blends; animated rotation, twist deformation, domain warping | Primitive SDFs: `sdSphere`, `sdCapsule`, `sdTorus`, `sdBox`; boolean ops: `min()` union, `max()` intersection, `−min()` subtraction; smooth union `smin(a,b,k) = -log(e^(-a/k)+e^(-b/k))·k`; twist `p.xz = rot2(p.y·twist_k)·p.xz`; domain repetition `p = mod(p,cell)-0.5·cell`; 3-point lighting (key + rim + moving point); normal via central differences; full AO + soft shadow pipeline from mandelbulb; 5 presets; 5 themes |
| 9 | `mandelbox_tunnel.c` | raymarcher/ | Infinite Corridor / Mandelbox Tunnel — repeating arched architecture, distance fog, constant forward camera drift; alien-architecture flythrough | Space repetition `p = fmod(p, period) - 0.5·period` for infinite pillars/arches; Mandelbox DE: box fold `clamp(p,-1,1)·2−p` + sphere fold `r<r_min → p/r_min²`, scale-and-translate; depth fog `exp(-t·fog_density)` dims distant geometry; vignette: radial falloff from screen center; camera: constant forward drift + slight sinusoidal yaw sway; progressive render; 5 themes |

---

## Tier 5 — Raytracer (analytic geometry)

Raytracer strength: sharp reflections, glass, Fresnel, refraction, multiple lights. Analytic intersection gives pixel-perfect geometry. Complements raymarcher (which excels at implicits).

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 10 | `glass_caustics.c` | raytracer/ | Glass Sphere Caustics — glass sphere, checker floor, two colored lights, mirror sphere; Fresnel reflection + refraction + shadow rays | Analytic sphere-ray intersection `t = -b ± √(b²−c)`; Fresnel `R = R0 + (1−R0)(1−cosθ)⁵` (Schlick); Snell's law refraction `sinθ_t = n1/n2·sinθ_i`; total internal reflection guard; recursive ray: reflect + refract at glass boundary (depth ≤ 4); checker floor `floor(x)+floor(z)` mod 2; shadow ray to each light; soft falloff shading `1/d²`; color accumulation per bounce; 5 themes |
| 11 | `reflection_hall.c` | raytracer/ | Torus + Capsule + Sphere Reflection Hall — mirror floor, three objects, slow orbit camera, 3-point lighting, specular sharpening, contact shadows | Analytic torus intersection via quartic solver (Newton iteration or Ferrari); capsule intersection via line-segment distance; mirror floor reflects full scene recursively (depth ≤ 3); 3-point lighting: key + fill + rim with per-light shadow rays; specular sharpening via high Phong exponent (64–128); contact shadows via short AO ray from hit point toward floor; camera orbits at constant radius; progressive render; 5 themes |
| 12 | `cornell_box.c` | raytracer/ | Cornell Box — classic renderer benchmark: red/blue walls, white ceiling/floor, sphere center, overhead area light; soft shadow; optional reflection/glass sphere | Axis-aligned box planes via slab intersection; area light: sample N_shadow points on light rect, average shadow; diffuse shading `Kd·(N·L)·light_color`; brightness → character ramp ` .:-=+*#%@`; optional mirror sphere (recursive reflect); optional glass sphere (Fresnel+Snell); color channels: R/G/B per wall rendered as distinct luminance via pair tinting; demonstrates light transport intuition; 5 themes |

---

## Tier 6 — Rasterizer (triangle pipeline)

Rasterizer strength: fast geometry animation, vertex deformation, mesh shading, wireframe overlays. Triangle pipeline maps directly to terminal cell grid — fast enough for real-time mesh animation.

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 13 | `flag_sim.c` | rasterizer/ | Waving Flag — grid plane mesh with sin-wave vertex displacement, wind direction, vertex normals recompute; Phong + toon + wireframe overlay modes | Grid mesh (W×H quads → 2 triangles each); displacement `y += A·sin(kx−ωt + wind_phase)`; per-vertex normal recompute: cross product of adjacent edge vectors; Phong: `Ka + Kd·(N·L) + Ks·(R·V)^shin`; toon: quantize `N·L` to 3 levels; wireframe: Bresenham edge draw over shaded triangles; barycentric rasterization for filled tris; depth buffer (1D per column) prevents z-fighting; keys toggle shading mode 1/2/3; 5 themes |
| 14 | `proc_mesh.c` | rasterizer/ | Procedural Mesh Viewer — sphere, torus, subdivided cube, icosphere; rotation, scaling pulse, lighting sweep; shading modes: normals / depth / wireframe / Phong | Procedural sphere: `(sinθ cosφ, cosθ, sinθ sinφ)` on UV grid; torus `((R+r cosφ)cosθ, r sinφ, (R+r cosφ)sinθ)`; icosphere via subdivision from tetrahedron; rotation matrix `Ry(t)·Rx(0.3t)`; normal visualisation: map N→[0,1]³ → RGB-like pair; depth vis: z-buffer value → char ramp; wireframe: edge list, Bresenham per edge; Phong: 3-point lighting; keys: `1` Phong, `2` normals, `3` depth, `4` wireframe; feels like mini OpenGL pipeline inspector; 5 themes |
| 15 | `terrain_fly.c` | rasterizer/ | Terrain Flyover — diamond-square or Perlin heightmap, forward flight camera, pitch/yaw drift, sun direction lighting, shadow banding, contour overlay | Diamond-square: recursive midpoint displacement `h_mid = avg(corners) + rand·scale; scale *= 0.5`; camera: constant forward `z -= speed·dt`, pitch `θ += pitch_input·dt`, yaw drift `φ += sin(t·0.3)·0.02`; perspective project `sx = f·x/z + cx`, clip z < near; flat shading per triangle: `N·sun_dir` → brightness; shadow banding: project vertex onto sun direction, compare to max height along ray for self-shadow; contour: draw horizontal isoline at fixed height intervals; height → char ramp ` .:-=+*#%@^`; 5 themes |

---

## Progress

**Done (2/15):** `barnes_hut`, `mandelbulb_explorer`

**Remaining (13):** `lbm`, `fluid_sph`, `fem_solid`, `rigid_body_rot`, `maxwell_fdtd`, `sdf_gallery`, `mandelbox_tunnel`, `glass_caustics`, `reflection_hall`, `cornell_box`, `flag_sim`, `proc_mesh`, `terrain_fly`

---

## Notes

### Tier 1 — Fluid
- **lbm.c** — grid maps 1:1 to terminal cells (no px_to_cell). τ = 0.6 is a stable starting point; τ < 0.5 diverges. Bounce-back at obstacle cells reverses all 9 f_i directions. Vorticity coloring with signed ramp directly reuses wave_interference color scheme.
- **fluid_sph.c** — hardest: spatial hash grid is critical for performance (O(1) vs O(N²) neighbour search). h (smoothing radius) ≈ 2–3 terminal cell widths in pixel space. Pressure constant k controls stiffness — too high = explosive, too low = compressible jelly.

### Tier 2 — Physics
- **fem_solid.c** — linear FEM is sufficient: assemble K once per topology change, solve Ku=f each frame for dynamic loading. Von Mises stress maps naturally to the existing 5-level color pair scheme.
- **rigid_body_rot.c** — builds directly on rigid_body.c: same AABB overlap solver, same two-pass resolution. Add angle/omega fields and rotational impulse denominator term. Rotated box draw via 4 corner px_to_cell conversions + line segments.

### Tier 3 — Algorithmic
- **barnes_hut.c** — static quadtree node pool (no malloc). θ = 0.5 is standard; lower = more accurate, higher = faster. Tree overlay visible at ~200 bodies before screen gets too dense.
- **maxwell_fdtd.c** — pairs with wave_interference (analytic) and schrodinger (QM wave): three wave physics programs together show classical EM, quantum, and geometric optics. PML boundary is essential — without it reflections corrupt the field within ~50 frames.

### Tier 4 — Raymarcher
- **mandelbulb_explorer.c** — progressive ROWS_PER_TICK=4 rendering keeps input responsive. Smooth coloring formula `mu = iter+1 - log(log(r)/log(bail))/log(p)` eliminates harsh color band boundaries. `g_dirty` only set on user input; morph mode intentionally avoids it to create radar-sweep scan effect.
- **sdf_gallery.c** — smooth union k controls blend sharpness: k=0.1 is subtle, k=1.0 is bulgy. Domain repetition works for objects but not for lighting — keep light computation in original space. Twist deformation applied in object space before SDF evaluation.
- **mandelbox_tunnel.c** — Mandelbox scale parameter (-2 to -1.5 for interesting tunnels). Forward camera speed must be tuned so the periodic structure feels infinite, not obviously looping. Depth fog strength controls claustrophobia vs open feel.

### Tier 5 — Raytracer
- **glass_caustics.c** — Fresnel at glass boundary is critical for realism; without it glass looks like a mirror. Limit recursion depth to 4 to prevent infinite loops. Two colored lights (warm + cool) create dramatic color contrast on the floor checker.
- **reflection_hall.c** — quartic torus intersection is numerically sensitive; Newton iteration with 8 steps at torus-AABB intersection point is reliable. Mirror floor needs depth limit or rays bounce forever. Contact shadows with ray length = 0.5 cell heights look realistic.
- **cornell_box.c** — area light sampling N=4 shadow rays is sufficient for soft penumbra in ASCII (pixel resolution hides noise). Red/blue walls can be represented by different color pairs rather than RGB; matches existing 256-color pair system.

### Tier 6 — Rasterizer
- **flag_sim.c** — wave propagation `kx−ωt` where k=2.5, ω=3.0 gives realistic flag motion. Vertex normals MUST be recomputed every tick since displacement changes the surface. Wireframe overlay drawn after filled triangles using depth check.
- **proc_mesh.c** — icosphere via 5-subdivision of tetrahedron gives 320 faces — enough for smooth silhouette in ASCII. Rotation matrix should be applied before perspective projection, not after. Normal visualization maps `N = 0.5·(N+1)` to [0,1] → color pair index.
- **terrain_fly.c** — diamond-square 129×129 grid (2^7+1) gives good terrain detail. Back-face culling essential for performance: skip triangles where `N·view_dir < 0`. Contour lines at fixed height intervals (every 8 cells) add topographic realism.
