# Next 6 Programs

Ideas ranked by learning value and visual payoff. Each builds on existing project knowledge.
All programs follow the §1–§8 framework: config → clock → color → coords → entity → scene → screen → app.
Physics in pixel space (CELL_W=8, CELL_H=16). Fixed timestep accumulator. Alpha render interpolation.
Sleep before render. erase → scene_draw → HUD → wnoutrefresh → doupdate.

---

## Tier 1 — Fluid Solvers

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 1 | `lbm.c` | fluid/ | Lattice Boltzmann D2Q9 — industry-standard grid fluid: density, velocity, vorticity fields with obstacle drag; presets: Kármán vortex street, lid-driven cavity, Poiseuille flow, double slit | D2Q9: 9 distribution functions f_i per cell; BGK collision `f_i' = f_i − (1/τ)(f_i − f_i^eq)`; equilibrium `f_i^eq = ρ w_i [1 + (c_i·u)/cs² + (c_i·u)²/2cs⁴ − u²/2cs²]`; streaming shifts f_i along c_i; ρ=Σf_i, u=Σf_i·c_i/ρ; vorticity `ω = ∂uy/∂x − ∂ux/∂y` → signed color ramp (same 8-level scheme as wave_interference); grid maps directly to terminal cells — no px_to_cell needed; obstacle cells use bounce-back (reverse direction); τ controls viscosity via `ν = cs²(τ−0.5)dt`; 5 themes |
| 6 | `fluid_sph.c` | fluid/ | Smoothed Particle Hydrodynamics — 2D particle fluid with pressure, viscosity, surface tension; splash simulation; dam break preset | N=800 particles in pixel space (CELL_W=8/CELL_H=16); poly6 density kernel `W(r,h)=(315/64πh⁹)(h²−r²)³`; spiky pressure gradient `∇W`; viscosity Laplacian `∇²W`; density `ρ_i=Σ m_j W(r_ij,h)`; pressure `P_i=k(ρ_i−ρ_0)`; spatial hash grid for O(1) neighbour search — each cell holds particle list; symplectic Euler integration; surface particles detected by density threshold → surface tension force; px_to_cell in scene_draw only; density → character density ramp (`.,:+#@`); velocity magnitude → color pair; 5 presets; 5 themes |

---

## Tier 2 — Physics Upgrades

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 2 | `fem_solid.c` | physics/ | Finite Element Method soft solid — 2D triangle mesh elasticity; beam bending, rubber plate compression, elastic cantilever; von Mises stress heatmap | Triangle mesh over terminal grid; per-element stiffness matrix `K_e = ∫ B^T D B dA` (B = strain-displacement, D = material matrix); global assembly `K_global u = f`; linear solve via Gauss elimination or Conjugate Gradient; displacement field → stress → von Mises `σ_v = √(σ_x²−σ_xσ_y+σ_y²+3τ²)`; stress magnitude → color pair + char density; fixed boundary nodes (Dirichlet BC); load applied via mouse/key to specific nodes; entity = mesh node array + connectivity; scene_tick assembles and solves per frame for dynamic loading; pixel space for node positions; px_to_cell in draw only; 5 themes |
| 5 | `rigid_body_rot.c` | physics/ | 2D rigid body with full angular momentum — torque, moment of inertia, spin integration; toppling towers, spinning cubes, rolling friction, angular collisions | Extends rigid_body.c: add `float angle, omega` (orientation + angular velocity) to Body struct; moment of inertia `I = (1/12)m(w²+h²)` box, `I = (1/2)mr²` disk; contact point `r = contact − centroid`; rotational impulse denominator `1/m_a + 1/m_b + (r_a×n)²/I_a + (r_b×n)²/I_b`; torque from friction `τ = r × F_friction`; spin integration `ω += τ/I · dt`; angle wraps mod 2π; draw rotated AABB as 4 corner lines using Bresenham; aspect-corrected corners `(hw, hh=2hw)` preserved; sleep threshold includes `|ω| < ω_sleep`; px_to_cell for all corner draws; fixed timestep accumulator unchanged from framework §8; 5 themes |

---

## Tier 3 — Algorithmic Simulation

| # | Name | Folder | Concept | Key Technique |
|---|---|---|---|---|
| 3 | `barnes_hut.c` | physics/ | Barnes–Hut O(n log n) gravity tree — 2000-body galaxy with quadtree acceleration; tree structure overlay; adaptive timestep; collision merging | Quadtree: each node holds `total_mass`, `cx`, `cy` (center of mass), bounding box; build tree every tick by inserting all N bodies; force on body i: traverse tree — if node is leaf or `s/d < θ` (θ=0.5), treat as point mass `F = G·m_i·M / d²`; else recurse into children; O(N log N) vs O(N²) naive; tree overlay: draw quadrant boundaries with ACS box chars at each occupied node level; bodies in pixel space (CELL_W=8/CELL_H=16); velocity Verlet integration; adaptive dt: scale by `1/|F_max|` to prevent fast-close encounters; collision merging when `d < r_i + r_j`: conserve momentum, remove smaller body; px_to_cell in scene_draw; quadtree node pool — static array, no malloc; 5 themes |
| 4 | `maxwell_fdtd.c` | physics/ | 2D Maxwell FDTD electromagnetic solver — TM mode (Ez, Hx, Hy); waveguide, diffraction slit, antenna emission, dielectric refraction; Perfectly Matched Layer absorbing boundary | Yee grid: Ez at cell centres, Hx/Hy at cell edges (staggered by half cell); update equations: `Ez[i][j] += dt/ε·(ΔHy/Δx − ΔHx/Δy)`, `Hx[i][j] -= dt/μ·ΔEz/Δy`, `Hy[i][j] += dt/μ·ΔEz/Δx`; CFL stability: `dt ≤ dx/(c√2)` — hardcoded in §1 config; PML: 8-cell absorbing boundary with conductivity ramp `σ(x) = σ_max·(x/d)²`; point source: additive sinusoidal injection `Ez[sy][sx] += sin(ω·t)`; dielectric slab: ε_r per cell stored in `float g_eps[ROWS][COLS]`; Ez → signed 8-level color ramp (CP_N3..CP_P3, same mapping as wave_interference); H field magnitude → secondary overlay; terminal grid IS the Yee grid — no px_to_cell; 4 presets; 5 themes |

---

## Progress

**Done (0/6):**

**Remaining (6):** lbm, fluid_sph, fem_solid, rigid_body_rot, barnes_hut, maxwell_fdtd

---

## Notes

- **lbm.c** — grid maps 1:1 to terminal cells (no px_to_cell). τ = 0.6 is a stable starting point; τ < 0.5 diverges. Bounce-back at obstacle cells reverses all 9 f_i directions. Vorticity coloring with signed ramp directly reuses wave_interference color scheme.
- **fluid_sph.c** — hardest: spatial hash grid is critical for performance (O(1) vs O(N²) neighbour search). h (smoothing radius) ≈ 2–3 terminal cell widths in pixel space. Pressure constant k controls stiffness — too high = explosive, too low = compressible jelly.
- **fem_solid.c** — linear FEM is sufficient: assemble K once per topology change, solve Ku=f each frame for dynamic loading. Von Mises stress maps naturally to the existing 5-level color pair scheme.
- **rigid_body_rot.c** — builds directly on rigid_body.c: same AABB overlap solver, same two-pass resolution. Add angle/omega fields and rotational impulse denominator term. Rotated box draw via 4 corner px_to_cell conversions + line segments.
- **barnes_hut.c** — static quadtree node pool (no malloc). θ = 0.5 is standard; lower = more accurate, higher = faster. Tree overlay visible at ~200 bodies before screen gets too dense.
- **maxwell_fdtd.c** — pairs with wave_interference (analytic) and schrodinger (QM wave): three wave physics programs together show classical EM, quantum, and geometric optics. PML boundary is essential — without it reflections corrupt the field within ~50 frames.
