# Pending Phase-3 Refactors

Folders below still need phase-3 learnability passes.  Phase 3 adds, on
top of the existing phase-1 file:
  • HOW TO READ THIS FILE   (reading order + naming + background)
  • GUIDED TUTORIAL         (5-8 numbered tutorials, first principles)
  • MENTAL MODEL block if absent

Already complete: Ai/, algorithms/, animation/, flocking/, matrix_rain/,
raster/, raymarcher/, raytracing/, robots/, turtle/.

---

## TODO

| Folder              | Files | Notes                                          |
|---------------------|------:|------------------------------------------------|
| artistic/           |    23 | aesthetic effects (galaxy, aurora, mandalas, hindu/islamic, plasma, etc.) — already mostly phase-1; some are large |
| fluid/              |    11 | CFD-related (Navier-Stokes, SPH, lattice gas, vorticity, shallow-water).  Note: reaction_diffusion + reaction_wave duplicates with procedural/fields/ — resolve before refactoring. |
| grids/              |    80 | 8 sub-folders (rect/hex/tri/polar × intro/showcase/placement).  Highly templated; consider whether tutorials are needed per-file or just one canonical reference per category. |
| particle_systems/   |    19 | fire, fireworks, smoke, rain, snow, comet, fountain, etc.  Note: constellation_learning is a duplicate of constellation. |
| physics/            |    36 | nbody, cloth, lorenz, double_pendulum, wave, schrodinger, magnetic_field, ising, etc. — large folder, broad scope |
| procedural/         |    69 | 6 sub-folders (chaos, fields, fractals, generational, patterns, worldgen) |
| signal/             |     4 | epicycles, fft_vis, fourier_art, fourier_draw — recently created folder |

Total: ~242 files still pending phase-3.

## Approach when resuming

For each folder:
1. Audit:  for f in folder/*.c; check for CONCEPTS / MENTAL MODEL / HOW
   TO READ / GUIDED TUTORIAL.  Most should have CONCEPTS already.
2. Batch by topic / complexity (3-5 files per batch).
3. Per file: add missing prose blocks; tutorials should reference
   sibling files (e.g. physics/cloth.c → animation/ragdoll_ropes.c
   for the Verlet contrast).
4. Compile-verify after each batch with -O2 -Wall -Wextra.

## Watch for

- Duplicates already identified:
  - particle_systems/constellation_learning.c  vs  constellation.c
  - geometry/delaunay_triangulation.c          vs  procedural/generational/delaunay_triangulation.c
  - fluid/reaction_diffusion.c + reaction_wave.c  vs  procedural/fields/reaction_diffusion_gray_scott.c

- grids/ is heavily templated — consider whether each file gets its
  own tutorial OR only the canonical "01_*" file in each sub-folder
  gets full phase-3 prose with the others using terse "see 01_* for
  the lesson" pointers.

- physics/ has 36 files spanning many sub-topics (oscillators, fluids,
  EM, QM, optics, magnetism).  Sub-batch by topic family.
