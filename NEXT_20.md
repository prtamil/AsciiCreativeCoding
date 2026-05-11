# Pending Phase-3 Refactors

Folders below still need phase-3 learnability passes.  Phase 3 adds, on
top of the existing phase-1 file:
  • HOW TO READ THIS FILE   (reading order + naming + background)
  • GUIDED TUTORIAL         (5-8 numbered tutorials, first principles)
  • MENTAL MODEL block if absent

Already complete: Ai/, algorithms/, animation/, flocking/, fluid/,
grids/, matrix_rain/, raster/, raymarcher/, raytracing/, robots/,
signal/, turtle/.

Note on grids/: completed via a different model than the rest of phase-3.
Instead of per-file HOW TO READ / GUIDED TUTORIAL prose, grids/ got:
  • a single grids/README.md (703 lines) covering the GridCtx + Pool
    abstraction, the drawing/placement taxonomies, reading order, and a
    full file index for all 76 files;
  • uniform refactor of all 76 files to the canonical shape (GridCtx,
    Cursor, Pool, ctx_*/cursor_*/pool_* APIs, PAIR_HUD/HINT, FPS_EWMA_ALPHA);
  • switch-case extraction across placement files so each per-mode case
    lives in its own named helper (~110 helpers extracted family-wide).
The CONCEPTS + MENTAL MODEL phase-1 blocks remain in every file.

---

## TODO

| Folder              | Files | Notes                                          |
|---------------------|------:|------------------------------------------------|
| artistic/           |    23 | aesthetic effects (galaxy, aurora, mandalas, hindu/islamic, plasma, etc.) — already mostly phase-1; some are large |
| particle_systems/   |    19 | fire, fireworks, smoke, rain, snow, comet, fountain, etc.  Note: constellation_learning is a duplicate of constellation. |
| physics/            |    36 | nbody, cloth, lorenz, double_pendulum, wave, schrodinger, magnetic_field, ising, etc. — large folder, broad scope |
| procedural/         |    69 | 6 sub-folders (chaos, fields, fractals, generational, patterns, worldgen) |

Total: ~147 files still pending phase-3.

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

- Duplicate audit (resolved 2026-05-11):
  - particle_systems/constellation_learning.c — TRUE duplicate of
    constellation.c (same physics + data; different line-drawer
    + reworded MENTAL MODEL).  DELETED; canonical kept.
  - geometry/delaunay_triangulation.c  ↔  procedural/generational/
    delaunay_triangulation.c — NOT duplicate: same Bowyer-Watson, but
    one is the algorithm showcase (with circumcircle verification
    phase), the other is the generative-art demo (12 seeds dropping,
    flashing edges).  KEPT BOTH; cross-references added in each header.
  - fluid/reaction_diffusion.c  ↔  procedural/fields/reaction_diffusion_
    gray_scott.c — same Gray-Scott, different framing: PDE / numerical-
    methods angle (fluid/, 7 presets, 9-pt Laplacian) vs pattern-field
    angle (procedural/, 5 presets, 5-pt Laplacian).  KEPT BOTH;
    cross-references added.
  - fluid/reaction_wave.c — NOT a duplicate of gray_scott.  Different
    PDE family (FitzHugh-Nagumo excitable medium, like nerve impulses
    and BZ chemistry, NOT Gray-Scott Turing patterns).  No action.

- grids/ pattern (one folder-level README + uniform refactor + per-mode
  helper extraction) may be a good template for procedural/ which has
  6 sub-folders with similar per-category templates.

- physics/ has 36 files spanning many sub-topics (oscillators, fluids,
  EM, QM, optics, magnetism).  Sub-batch by topic family.
