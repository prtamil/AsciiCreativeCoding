/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cube_raster.c — software rasteriser, cube demo
 *
 * DEMO: A textured glossy cube (warm orange) tumbling on a black
 *       background. Cycle through phong → toon → normals → wire to
 *       see four shading models running on the same triangle mesh.
 *       Press 'c' to toggle back-face culling and watch the inside
 *       faces appear; press '+' / '-' to zoom.
 *
 *       This is the smallest possible software rasteriser:
 *           - 24 vertices (6 faces × 4 corners, flat-normal duplicated)
 *           - 12 triangles (each face is a quad split into 2 tris)
 *           - 4 shaders pluggable as function pointers
 *           - z-buffer, back-face cull, barycentric rasteriser
 *           - continuous RGB pipeline → 6×6×6 cube + 92-char Bourke ramp
 *
 * Study alongside:
 *   raster/sphere_raster.c           — same skeleton, smooth normals
 *   raster/torus_raster.c            — same skeleton, parametric tessellation
 *   raster/displace_raster.c         — same skeleton + vertex displacement
 *   raster/deferred_rendering_pipeline.c — same skeleton + G-buffer
 *
 * Section map:
 *   §1 config        — frame rate, FOV, cube geometry, ramp, ncurses pairs
 *   §2 clock         — monotonic timer + sleep
 *   §3 math          — V3, V4, Mat4 (transform matrices)
 *   §4 paint         — 216-pair RGB cube + Bourke ramp + paint_cell
 *   §5 shaders       — VS/FS function-pointer interface + 4 shader pairs
 *                      §5.1 types     §5.2 default vertex shader
 *                      §5.3 phong     §5.4 toon
 *                      §5.5 normals   §5.6 wire (barycentric edge detection)
 *   §6 mesh          — cube tessellation (24 vertices, 12 triangles)
 *   §7 framebuffer   — z-buffer + cell buffer + clear/blit
 *   §8 pipeline      — barycentric + per-triangle rasteriser
 *                      §8.1 barycentric (Möller signed-area form)
 *                      §8.2 pipeline_draw_mesh (vert → cull → raster → frag)
 *   §9 scene         — Uniforms, scene_tick, shader swap
 *   §10 screen       — ncurses init + HUD (yellow row 0 + cyan bottom)
 *   §11 app          — signals, resize, fixed-step main loop
 *
 * Keys:
 *   s         cycle shader (phong → toon → normals → wire)
 *   c         toggle back-face culling
 *   space     pause / resume rotation
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raster/cube_raster.c -o cube_rt -lncurses
 * -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Software rasterisation in the classic 1990s pipeline:
 *                  vertex shader → perspective divide → screen mapping
 *                  → back-face cull → barycentric rasteriser → fragment
 *                  shader → z-test → write framebuffer cell.
 *
 *                  The cube is a TEACHING choice: 6 flat faces with
 *                  duplicated vertices (4 per face) make the shading
 *                  model visible as hard steps — phong shows uniform
 *                  diffuse per face, toon shows hard cel boundaries
 *                  between faces, normals shows 6 distinct face colours,
 *                  wire shows the 12-edge silhouette via barycentric
 *                  edge detection.
 *
 *                  The output pipeline is identical to the raytracing
 *                  folder's: continuous RGB → Reinhard tone-map → gamma
 *                  → 6×6×6 cube + 92-char Bourke ramp + A_BOLD/A_DIM.
 *                  This unifies the rendering across every demo in the
 *                  raster + raytracer folders.
 *
 * Data-structure : `Mesh` = static array of `Vertex` (pos, normal, uv) +
 *                  static array of `Triangle` (3 indices). Cube has 24
 *                  vertices, 12 triangles, allocated ONCE at startup.
 *                  `Framebuffer` = z-buffer (float per cell, init +∞) +
 *                  colour buffer (Cell per cell: char + pair + attr).
 *                  Reset between frames with `fb_clear`.
 *                  Shaders are FUNCTION POINTERS — plug-and-play; the
 *                  pipeline doesn't know what they do.
 *
 * Rendering      : Per fragment: V3 colour from frag shader → Reinhard
 *                  tone-map per channel → gamma 1/2.2 encode → quantise
 *                  to 216-cell xterm cube → density char from Bourke
 *                  92-char ramp by Rec.709 luma. Bayer 4×4 dither
 *                  perturbs luma slightly so neighbouring cells with
 *                  similar brightness pick different glyphs (smoother
 *                  gradients on coarse grid).
 *
 * Performance    : 12 triangles · ~rows×cols rasterisation cells per
 *                  frame. Bottleneck is per-fragment cost (4 luma
 *                  computes + dither + cube quantise + ramp lookup).
 *                  At 240×80 cells × 60 fps ≈ 4 ms shading per frame.
 *
 * References     : Real-Time Rendering 4e §23 (software rasterisation).
 *                  Möller, "Fast triangle rasterisation by interpolating
 *                    edge functions," Game Programming Gems (2000).
 *                    The signed-area barycentric form used in §8.1.
 *                  Foley, van Dam, Feiner & Hughes, "Computer Graphics:
 *                    Principles and Practice" 3e §10.4 (rasterisation
 *                    fundamentals).
 *                  Reinhard et al., "Photographic Tone Reproduction for
 *                    Digital Images," SIGGRAPH '02.
 *                  Tanner Helland (blackbody) — used elsewhere in the
 *                    raytracer folder; unrelated here but the same
 *                    paint pipeline is shared.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A unit cube is 24 vertices and 12 triangles. Each frame:
 *   1. Build a model matrix (rotate X, then Y); compose with view (look
 *      at origin) and projection (perspective) → MVP matrix.
 *   2. For each triangle:
 *        - Run the vertex shader on its 3 vertices (transform to clip).
 *        - Perspective-divide to NDC; map to screen.
 *        - Reject if back-facing (signed area ≤ 0).
 *        - Rasterise: walk the triangle's screen-space bounding box;
 *          for each pixel inside (all 3 barycentric coords > 0), z-test;
 *          run the fragment shader on the interpolated state to get RGB;
 *          write to framebuffer cell.
 *   3. After all 12 triangles: blit the framebuffer to ncurses.
 *
 * It's a tiny GPU written in C. Vertices are STAMPS; transform them to
 * where they should appear on screen. Triangles are PAINT MASKS defined
 * by 3 stamps. For every cell under a paint mask, ask "what's my
 * position WITHIN this mask?" — that's barycentric coordinates u, v, w.
 * Use those to interpolate any per-vertex attribute (normal, UV, world
 * position) to the current pixel. Hand the interpolated state to a
 * fragment shader → RGB → cube cell.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *
 *      MODEL → VIEW → PROJ
 *        ↓        ↓       ↓     ← matrices composed into MVP
 *       (vertex shader runs here)
 *        ↓
 *      CLIP-SPACE TRIANGLE
 *        ↓
 *      perspective divide → SCREEN-SPACE TRIANGLE
 *        ↓
 *      back-face cull (signed area)
 *        ↓
 *      bounding-box raster:
 *        for each pixel:
 *          barycentric (u, v, w)
 *          if any < 0 → skip (outside)
 *          z = u·z0 + v·z1 + w·z2
 *          if z >= zbuf → skip (occluded)
 *          interpolate position, normal, uv
 *          (fragment shader runs here)
 *          → V3 RGB → tone-map → cube cell
 *
 * Same skeleton as the rest of the raster folder; only §6 (mesh
 * tessellation) differs between cube/sphere/torus/displaced.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────────────────
 *  1. Tessellate cube ONCE at startup: 6 faces × 4 vertices = 24
 *     vertices, each with face-local outward normal (so faces have
 *     uniform normals — flat shading, hard edges).
 *  2. scene_tick: angle_x += ROT_X·dt, angle_y += ROT_Y·dt;
 *     model = rot_y(angle_y) · rot_x(angle_x);
 *     mvp   = proj · view · model;
 *     norm_mat = cofactor(model)  (correct normal transform under
 *                                   non-uniform scale).
 *  3. fb_clear: zbuf[i] = +∞, cbuf[i] = empty.
 *  4. For each of 12 triangles:
 *       a. vert_shader on 3 vertices → clip_pos, world_pos, world_nrm.
 *       b. Near-clip reject (all w < ε).
 *       c. Perspective divide:
 *            sx = ( clip.x/w + 1)·0.5·cols
 *            sy = (-clip.y/w + 1)·0.5·rows  (Y-flip for screen-down)
 *            sz =   clip.z/w
 *       d. Signed area; if cull_backface AND area ≤ 0 → skip.
 *       e. Compute bbox; walk every pixel in it.
 *       f. For each pixel: barycentric → all > 0 + z-test → run fragment.
 *       g. fragment shader returns V3 RGB; paint_cell tones, gammas,
 *          quantises to cube + ramp char.
 *  5. fb_blit to stdscr; HUD; present.
 *
 * KEY FORMULAS
 * ────────────
 *  Perspective projection (OpenGL-style):
 *     m[0][0] = (1/tan(fov/2)) / aspect
 *     m[1][1] =  1/tan(fov/2)
 *     m[2][2] = (far+near)/(near-far)
 *     m[2][3] = (2·far·near)/(near-far)
 *     m[3][2] = -1
 *
 *  Aspect with terminal cell ratio:
 *     aspect = (cols·CELL_W) / (rows·CELL_H)   CELL_W=8, CELL_H=16
 *  Without this the cube renders as a vertical ellipsoid because
 *  terminal cells are taller than wide.
 *
 *  Perspective divide + screen mapping:
 *     sx = ( clip.x / w + 1) · 0.5 · cols      NDC x ∈ [-1,1]
 *     sy = (-clip.y / w + 1) · 0.5 · rows      Y-flip for screen-down
 *     sz =   clip.z / w
 *
 *  Barycentric (Möller signed-area form):
 *     d  = (y1−y2)(x0−x2) + (x2−x1)(y0−y2)
 *     b0 = ((y1−y2)(px−x2) + (x2−x1)(py−y2)) / d
 *     b1 = ((y2−y0)(px−x2) + (x0−x2)(py−y2)) / d
 *     b2 = 1 − b0 − b1
 *
 *  Back-face cull (after Y-flip, CCW = positive area):
 *     area = (sx1−sx0)(sy2−sy0) − (sx2−sx0)(sy1−sy0)
 *     cull when area ≤ 0
 *
 *  Blinn-Phong (frag_phong):
 *     N=world normal, L=light_dir, V=view_dir, H=normalize(L+V)
 *     diff = max(0, N·L)
 *     spec = max(0, N·H)^shininess
 *     col  = ambient + obj·light·diff + spec·0.5
 *     gamma = col^(1/2.2)
 *
 *  Toon (frag_toon, 4 bands):
 *     banded = floor(diff·bands) / bands
 *     spec   = (N·H > 0.94) ? 0.7 : 0       hard cel highlight
 *
 *  Normals (frag_normals):  RGB = N·0.5 + 0.5  ∈ [0,1] per channel
 *
 *  Wireframe (frag_wire):
 *     edge = min(b0, b1, b2)    distance to nearest edge
 *     if edge > WIRE_THRESH → discard
 *     else colour ramp from white at edge to grey at threshold
 *
 *  Bayer 4×4 ordered dither (k_bayer[py%4][px%4]):
 *      0  8  2 10                /16
 *     12  4 14  6
 *      3 11  1  9
 *     15  7 13  5
 *
 *  RGB → 6×6×6 cube pair (xterm 16..231):
 *     r5 = clamp(round(r·5), 0, 5);   g5, b5 likewise
 *     pair_id = PAIR_CUBE_BASE + r5·36 + g5·6 + b5
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Y-FLIP. NDC has +Y up but screen rows go down. The minus sign in
 *    `sy = (-clip.y/w + 1)·…` flips it. Without the flip the cube is
 *    upside-down AND back-face culling inverts (CCW becomes CW after
 *    flip).
 *  • BACK-FACE AREA SIGN. After the Y-flip, CCW triangles have POSITIVE
 *    area; cull when `area <= 0` (also catches zero-area degenerates).
 *  • WIREFRAME IDENTITY COORDS. The pipeline injects (1,0,0), (0,1,0),
 *    (0,0,1) into VSIn.u/v BEFORE the vertex shader; vert_wire writes
 *    to custom[0..2] AFTER. Skip either step and barycentric edge
 *    detection in frag_wire reads zeros and the whole triangle is
 *    discarded.
 *  • DUPLICATED CUBE VERTICES. Each face has its own 4 vertices with
 *    a uniform face-normal — 24 total. Sharing vertices between faces
 *    would average normals at corners and round the cube. Wrong for
 *    hard-surface shading.
 *  • NEAR-CLIP. Triangles entirely behind the camera (all w < ε) are
 *    skipped before perspective divide. Triangles partially behind
 *    are NOT properly clipped (would require Sutherland-Hodgman) and
 *    can produce stretched artefacts when CAM_DIST is set very small.
 *  • BOURKE RAMP BOUNDS. Indexing past `BOURKE_LEN−1` reads beyond the
 *    string literal. Always clamp `idx` after the cast.
 *  • DITHER SCALE. `0.15` is chosen so dither doesn't smear the toon's
 *    hard bands. Larger values dissolve cel-shading; smaller values
 *    posterise phong gradients.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PHONG shader: light at (4, 5, 4); lit faces are upper-front-right;
 *    opposite faces are dim ambient grey. Specular highlight moves
 *    smoothly across the front face as the cube rotates.
 *  • TOON shader: 4 bands. Each face is entirely ONE band — no
 *    gradient across one face (flat normals).
 *  • NORMALS shader: 6 distinct face colours. +X → reddish, +Y →
 *    greenish, +Z → bluish (after the ·0.5+0.5 shift), opposites use
 *    complementary colours.
 *  • WIRE: 12 edges visible plus the 6 diagonals from quad-to-tri
 *    splits. As the cube rotates you can count edges.
 *  • CULL TOGGLE 'c': switch off — interior face glyphs appear inside
 *    the cube outline. Switch on — clean silhouette.
 *  • ZOOM +/-: cube grows/shrinks; HUD shows current cam distance
 *    (1.0..8.0 with 0.2 step). Aspect remains correct because proj
 *    is rebuilt with cell-aspect on resize.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. This is the SIMPLEST full triangle rasteriser in
 *      this folder; read it first if you want to understand the
 *      raster pipeline before tackling sphere / torus / shadow / SSAO.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §3 math — V3, V4, Mat4 + matrix builders. Read AFTER tutorial T2.
 *   4. §6 mesh — cube tessellation (24 verts, 12 tris). Skim.
 *   5. §8 pipeline — barycentric + per-triangle rasteriser. Read AFTER
 *      tutorials T3-T6.
 *   6. §5 shaders — four VS/FS pairs (phong/toon/normals/wire) plug
 *      into the pipeline via function pointers. Read AFTER tutorial T7.
 *   7. §4 paint + §7 framebuffer + §9-§11 — infrastructure; skim.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   V3 / V4 / Mat4   3-D vector / 4-D vector / 4×4 matrix
 *   model / view /   object → world / world → camera / camera → clip
 *     proj / mvp     transform matrices, composed into mvp
 *   norm_mat         transposed-inverse of model for transforming normals
 *   sx / sy / sz     screen-space coordinates (after perspective divide)
 *   b0 / b1 / b2     barycentric weights (sum to 1; all > 0 = inside)
 *   zbuf / cbuf      z-buffer (depth) and colour buffer (Cell)
 *   VSIn / VSOut     vertex shader input / output
 *   FSIn             fragment shader input (interpolated VSOut)
 *
 * Background you need
 * ───────────────────
 *   - 4×4 matrices for transforms (identity, perspective, lookAt).
 *   - Homogeneous coordinates: a 3-D point becomes (x, y, z, 1) so
 *     translation can be a matrix multiplication.
 *   - Cross product and signed area of a triangle.
 *   - Linear interpolation: a · b0 + b · b1 + c · b2 with b0+b1+b2=1.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - GPU shading languages — these are CPU function pointers.
 *   - Sutherland-Hodgman near-plane clipping (we just reject whole
 *     triangles where all vertices are behind near; partial clipping
 *     not implemented).
 *   - Texture sampling — cube faces are flat-coloured by face-normal.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Eight short tutorials that build the rasteriser from first
 * principles. Read in order; each builds on the previous.
 *
 *   T1  The 7-stage raster pipeline — what every triangle goes through
 *   T2  MVP matrix — composing model · view · proj
 *   T3  Perspective divide + screen mapping (NDC → cells)
 *   T4  Back-face culling via signed area
 *   T5  Barycentric coordinates (Möller signed-area form)
 *   T6  Z-buffer — hidden surface removal in O(1) per pixel
 *   T7  Four shader pairs — phong / toon / normals / wire
 *   T8  Continuous-RGB pipeline → 6×6×6 cube + Bayer dither + ramp
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE 7-STAGE RASTER PIPELINE
 * ────────────────────────────────
 * Every triangle in this file goes through the same seven stages:
 *
 *   STAGE 1  VERTEX SHADER       per-vertex transform: object-space
 *                                 vertex → clip-space position +
 *                                 carried-along data (world pos, normal,
 *                                 UV) for the fragment shader.
 *
 *   STAGE 2  PERSPECTIVE DIVIDE  divide x, y, z by w. Result is in
 *                                 normalised device coordinates (NDC),
 *                                 each axis in [-1, +1].
 *
 *   STAGE 3  SCREEN MAPPING      NDC → cell coordinates (sx, sy, sz).
 *                                 Y is flipped (NDC up = +1 → screen
 *                                 row 0 = top).
 *
 *   STAGE 4  BACK-FACE CULL      compute signed triangle area in screen
 *                                 space. CCW = positive (front);
 *                                 CW = negative (back, cull if enabled).
 *
 *   STAGE 5  BOUNDING-BOX RASTER walk every cell in the triangle's
 *                                 axis-aligned screen bbox. For each:
 *                                   compute barycentric (T5)
 *                                   if any < 0 → outside, skip
 *
 *   STAGE 6  Z-TEST (HSR)         interpolated z = b0·z0 + b1·z1 + b2·z2.
 *                                 If z >= zbuf[cell] → occluded, skip.
 *                                 Else: zbuf[cell] = z, proceed.
 *
 *   STAGE 7  FRAGMENT SHADER      use barycentrics to interpolate
 *                                 normal/UV/world position to this
 *                                 pixel; FS returns RGB; paint_cell
 *                                 quantises to cube + ramp.
 *
 * Read §8 pipeline_draw_mesh — it implements stages 1-6 directly.
 * Stage 7 is the FS function pointer the user picks via 's'.
 *
 * T2  MVP MATRIX — COMPOSING MODEL · VIEW · PROJ
 * ───────────────────────────────────────────────
 * Three transforms compose to put a vertex on the screen:
 *
 *     v_clip = PROJ · VIEW · MODEL · v_object
 *              ╲___╱   ╲___╱   ╲____╱   ╲_______╱
 *               5         4       3        2        ← reading order
 *
 *   1. v_object — vertex's local-space coords (e.g. cube corner at
 *                 (0.5, 0.5, 0.5)).
 *   2. MODEL    — places the object in the WORLD. Rotation,
 *                 translation, scale.
 *   3. VIEW     — places the camera. Mat4 from `lookAt(eye, target, up)`
 *                 — equivalent to "translate so camera is at origin
 *                 then rotate so it looks down -Z".
 *   4. PROJ     — perspective. Maps 4-D homogeneous coords such that
 *                 the camera frustum becomes a clip box. After
 *                 perspective divide (T3), points in front of the
 *                 camera land in NDC's [-1, +1]³.
 *   5. CLIP     — final 4-D output. Per vertex.
 *
 * In code (§9 scene_tick):
 *
 *     model = rot_y(angle_y) · rot_x(angle_x);
 *     view  = lookAt(camera, (0,0,0), up);
 *     mvp   = proj · view · model;       composed once per frame
 *
 * Per-vertex (§5.2 default vertex shader):
 *
 *     v_clip = mvp · v_object;
 *
 * Normals need a separate matrix because non-uniform scale would
 * shear them — `norm_mat = transpose(inverse(model))`. For pure
 * rotation/uniform scale (our case) this equals the model matrix's
 * 3×3 upper-left.
 *
 * T3  PERSPECTIVE DIVIDE + SCREEN MAPPING (NDC → CELLS)
 * ──────────────────────────────────────────────────────
 * After the vertex shader, we have v_clip = (cx, cy, cz, cw). The
 * cw component encodes "depth from camera" — a vertex 5 m away has
 * cw ≈ 5. Perspective divide converts to NDC:
 *
 *     ndc.x = cx / cw     ∈ [-1, +1] inside the visible frustum
 *     ndc.y = cy / cw
 *     ndc.z = cz / cw
 *
 * That divide IS the perspective effect — distant vertices (large
 * cw) get small ndc values and cluster near the centre.
 *
 * Then we map NDC to cell coordinates:
 *
 *     sx = ( ndc.x + 1) · 0.5 · cols       NDC -1 → 0; +1 → cols
 *     sy = (-ndc.y + 1) · 0.5 · rows       Y FLIP: NDC +1 (up) → row 0
 *     sz = ndc.z                            kept for z-buffer
 *
 * The Y-flip is critical. NDC has +Y up; screen rows go DOWN. Without
 * the minus sign, the cube renders upside-down — AND back-face
 * culling inverts (CCW becomes CW after flip). Cull-condition then
 * needs adjusting: post-flip, CCW = positive area = front face
 * (T4).
 *
 * Near-clip rejection: if all three vertices have cw < ε, the
 * triangle is entirely behind the near plane — skip BEFORE perspective
 * divide (dividing by zero or negative cw produces garbage).
 *
 * T4  BACK-FACE CULLING VIA SIGNED AREA
 * ──────────────────────────────────────
 * After screen mapping we have three 2-D points (sx0,sy0), (sx1,sy1),
 * (sx2,sy2). The signed area of the triangle they form is:
 *
 *     area = (sx1 - sx0)(sy2 - sy0) - (sx2 - sx0)(sy1 - sy0)
 *
 * Sign tells us orientation:
 *
 *     area > 0   counter-clockwise (CCW) — front face after Y-flip
 *     area = 0   degenerate (zero-area triangle, skip)
 *     area < 0   clockwise (CW)         — back face, cull if enabled
 *
 * The convention "CCW = front" is OpenGL's. After the Y-flip in
 * stage 3, our screen-space triangles inherit it.
 *
 * Pseudocode:
 *     if cull_enabled and area <= 0: skip triangle
 *
 * Saves ~half the per-pixel work on a closed mesh — every back-
 * facing triangle is invisible from this camera angle anyway. Press
 * 'c' to toggle and see interior faces appear.
 *
 * T5  BARYCENTRIC COORDINATES (MÖLLER SIGNED-AREA FORM)
 * ──────────────────────────────────────────────────────
 * For a point P inside triangle (V0, V1, V2), barycentric coordinates
 * (b0, b1, b2) are the WEIGHTS such that:
 *
 *     P = b0·V0 + b1·V1 + b2·V2,      b0 + b1 + b2 = 1
 *
 * Geometrically: b0 is the fractional area of the sub-triangle
 * opposite V0 (between P, V1, V2). Same for b1 (opposite V1) and
 * b2 (opposite V2). All three positive → P inside.
 *
 * Computed via the signed-area form (Möller 2000):
 *
 *     d  = (y1-y2)(x0-x2) + (x2-x1)(y0-y2)         total area · 2
 *     b0 = ((y1-y2)(px-x2) + (x2-x1)(py-y2)) / d
 *     b1 = ((y2-y0)(px-x2) + (x0-x2)(py-y2)) / d
 *     b2 = 1 - b0 - b1
 *
 * d is computed ONCE per triangle (constant). b0, b1 are computed
 * per-pixel. The whole inside-test is:
 *
 *     if b0 >= 0 and b1 >= 0 and b2 >= 0: inside
 *
 * Once inside, barycentrics interpolate any vertex attribute:
 *
 *     z       = b0·z0 + b1·z1 + b2·z2          (for z-buffer)
 *     normal  = b0·n0 + b1·n1 + b2·n2          (for fragment shader)
 *     world_p = b0·p0 + b1·p1 + b2·p2
 *
 * That's the entire triangle rasteriser. Walk the bbox, compute
 * three weights, test inside, interpolate, ship to fragment shader.
 *
 * T6  Z-BUFFER — HIDDEN SURFACE REMOVAL IN O(1) PER PIXEL
 * ────────────────────────────────────────────────────────
 * Multiple triangles can cover the same pixel — we want only the
 * NEAREST one to win. Z-buffer is the standard solution:
 *
 *     for each pixel:
 *       initialise zbuf[pixel] = +∞
 *
 *     when a triangle wants to write to (x, y) with depth z:
 *       if z >= zbuf[x][y]: skip       occluded by previous closer
 *       else:               write,
 *                           zbuf[x][y] = z   record new winner
 *
 * Cost: O(1) per pixel per triangle. The triangles can be processed
 * in ANY order — each pixel independently keeps the nearest winner
 * regardless of submission order.
 *
 * Limitations:
 *   - Memory: one float per cell. Trivial at terminal resolution.
 *   - Transparency: doesn't work (needs back-to-front sort).
 *   - Anti-aliasing: a pixel either passes or fails z-test; no
 *     partial coverage.
 *
 * For an opaque cube, z-buffer is perfect. Read §7 (zbuf, cbuf,
 * fb_clear) and §8 (z-test inside the per-pixel loop).
 *
 * T7  FOUR SHADER PAIRS — PHONG / TOON / NORMALS / WIRE
 * ──────────────────────────────────────────────────────
 * The pipeline is shape-agnostic and shader-agnostic. The user's
 * `s' key cycles between four (VS, FS) pairs, all plugging into the
 * same pipeline:
 *
 *   PHONG    smooth diffuse + Blinn-Phong specular. Realistic.
 *            Lit faces show smooth gradient; opposite faces dim.
 *
 *   TOON     diffuse quantised into 4 bands (cel-shading) + a
 *            hard binary specular cut-off at N·H > 0.94. Each
 *            cube face = ONE band (flat normals).
 *
 *   NORMALS  RGB = (N + 1) / 2 — surface normal as colour. Each
 *            face is a flat distinct colour (+X red-ish, +Y green-
 *            ish, etc). Diagnostic for "are normals correct?"
 *
 *   WIRE     barycentric edge detection: edge_distance =
 *            min(b0, b1, b2). Where edge_dist > WIRE_THRESH
 *            DISCARD the fragment; else fade colour from white at
 *            edge to dim near threshold. 12 cube edges + 6 quad-
 *            split diagonals visible.
 *
 * The vertex shader is mostly the SAME for all four — it just
 * computes clip_pos, world_pos, world_normal. WIRE adds a special
 * vertex shader that writes barycentric identity coords (1,0,0),
 * (0,1,0), (0,0,1) into the FSIn.custom slot — those interpolate
 * across the triangle, giving the FS the per-pixel barycentric
 * needed for edge detection.
 *
 * Read §5 — each shader is ~20 lines.
 *
 * T8  CONTINUOUS-RGB PIPELINE → 6×6×6 CUBE + DITHER + RAMP
 * ─────────────────────────────────────────────────────────
 * Every fragment shader returns a CONTINUOUS V3 RGB in [0, ∞)³.
 * paint_cell quantises that to a terminal cell:
 *
 *   1. Reinhard tone-map per channel:  L' = L / (1 + L)
 *      Compresses HDR (specular peaks > 1.0) into [0, 1).
 *   2. Gamma encode 1/2.2: brightens midtones perceptually.
 *   3. Bayer 4×4 dither: add (dither[r%4][c%4] - 0.5) · 0.15 to luma
 *      before glyph index lookup. This perturbs brightness slightly
 *      so neighbouring cells with similar luma pick DIFFERENT ramp
 *      glyphs — smoother gradients on a coarse 6×6×6 cube.
 *   4. Quantise: r5 = round(r · 5) ∈ [0, 5] per channel.
 *   5. Pair index = 16 + 36·r5 + 6·g5 + b5  (xterm cube convention).
 *   6. Density char = ramp[round(luma_dithered · 91)] in 92-char
 *      Bourke ramp.
 *   7. A_BOLD on luma > 0.85, A_DIM on luma < 0.15.
 *
 * Bayer dither is the single biggest visual quality improvement
 * over plain quantisation. Without it, a smooth phong gradient
 * shows visible bands at every cube-cell boundary. With it, the
 * boundaries dissolve into noise that the eye averages out.
 *
 * Read §4 paint_cell.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <float.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate */
enum {
  FPS_TARGET = 60,
  FPS_UPDATE_MS = 500,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define DT_CAP_NS (100 * NS_PER_MS)

/* §1.2 view geometry */
#define CAM_FOV (55.0f * 3.14159265f / 180.0f)
#define CAM_NEAR 0.1f
#define CAM_FAR 100.0f
#define CAM_DIST 2.8f /* default camera Z distance              */
#define CAM_DIST_MIN 1.0f
#define CAM_DIST_MAX 8.0f
#define CAM_ZOOM_STEP 0.2f

/* CELL_W, CELL_H — terminal cell aspect for projection.
 * Without aspect correction the cube renders as a vertical ellipsoid. */
#define CELL_W 8
#define CELL_H 16

/* §1.3 cube geometry */
#define CUBE_S 0.75f /* half-extent: vertices at ±CUBE_S       */

/* §1.4 rotation rates (rad/sec) — different X and Y for tumbling. */
#define ROT_Y 0.55f
#define ROT_X 0.37f

/* §1.5 wireframe edge threshold (barycentric distance). */
#define WIRE_THRESH 0.06f

/* §1.6 Bourke 92-char ASCII density ramp (sparse → dense). */
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define BOURKE_LEN ((int)(sizeof k_bourke - 1))

/* §1.7 Bayer 4×4 ordered dither matrix (values in [0, 1)). */
static const float k_bayer[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};

#define DITHER_AMP 0.12f

/* §1.8 ncurses pair IDs. */
#define PAIR_CUBE_BASE 1 /* + 0..215 = 6×6×6 RGB cube              */
#define PAIR_HUD 217     /* yellow status row 0                    */
#define PAIR_HINT 218    /* cyan hint bottom row                   */

/* ── §2 clock ────────────────────────────────────────────────────────── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {.tv_sec = (time_t)(ns / NS_PER_SEC),
                       .tv_nsec = (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 math (V3, V4, Mat4) ──────────────────────────────────────────── */

typedef struct {
  float x, y, z;
} Vec3;
typedef struct {
  float x, y, z, w;
} Vec4;
typedef struct {
  float m[4][4];
} Mat4;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3_add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3_sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3_scale(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}
static inline float v3_dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
static inline Vec3 v3_norm(Vec3 a) {
  float l = v3_len(a);
  return l > 1e-7f ? v3_scale(a, 1.f / l) : v3(0, 1, 0);
}
static inline Vec3 v3_bary(Vec3 a, Vec3 b, Vec3 c, float u, float v, float w) {
  return v3(u * a.x + v * b.x + w * c.x, u * a.y + v * b.y + w * c.y,
            u * a.z + v * b.z + w * c.z);
}

static inline Vec4 v4(float x, float y, float z, float w) {
  return (Vec4){x, y, z, w};
}

static inline Mat4 m4_identity(void) {
  Mat4 m = {{{0}}};
  m.m[0][0] = m.m[1][1] = m.m[2][2] = m.m[3][3] = 1.f;
  return m;
}

static inline Vec4 m4_mul_v4(Mat4 m, Vec4 v) {
  return v4(
      m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z + m.m[0][3] * v.w,
      m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z + m.m[1][3] * v.w,
      m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z + m.m[2][3] * v.w,
      m.m[3][0] * v.x + m.m[3][1] * v.y + m.m[3][2] * v.z + m.m[3][3] * v.w);
}

static inline Mat4 m4_mul(Mat4 a, Mat4 b) {
  Mat4 r = {{{0}}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++)
        r.m[i][j] += a.m[i][k] * b.m[k][j];
  return r;
}

static inline Vec3 m4_pt(Mat4 m, Vec3 p) {
  Vec4 r = m4_mul_v4(m, v4(p.x, p.y, p.z, 1.f));
  return v3(r.x, r.y, r.z);
}

static inline Vec3 m4_dir(Mat4 m, Vec3 d) {
  Vec4 r = m4_mul_v4(m, v4(d.x, d.y, d.z, 0.f));
  return v3(r.x, r.y, r.z);
}

static Mat4 m4_rotate_y(float a) {
  Mat4 m = m4_identity();
  m.m[0][0] = cosf(a);
  m.m[0][2] = sinf(a);
  m.m[2][0] = -sinf(a);
  m.m[2][2] = cosf(a);
  return m;
}

static Mat4 m4_rotate_x(float a) {
  Mat4 m = m4_identity();
  m.m[1][1] = cosf(a);
  m.m[1][2] = -sinf(a);
  m.m[2][1] = sinf(a);
  m.m[2][2] = cosf(a);
  return m;
}

static Mat4 m4_perspective(float fovy, float aspect, float near, float far) {
  Mat4 m = {{{0}}};
  float f = 1.f / tanf(fovy * 0.5f);
  m.m[0][0] = f / aspect;
  m.m[1][1] = f;
  m.m[2][2] = (far + near) / (near - far);
  m.m[2][3] = (2.f * far * near) / (near - far);
  m.m[3][2] = -1.f;
  return m;
}

static Mat4 m4_lookat(Vec3 eye, Vec3 at, Vec3 up) {
  Vec3 f = v3_norm(v3_sub(at, eye));
  Vec3 r = v3_norm(v3(f.z * up.y - f.y * up.z, f.x * up.z - f.z * up.x,
                      f.y * up.x - f.x * up.y));
  Vec3 u =
      v3(r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x);
  Mat4 m = m4_identity();
  m.m[0][0] = r.x;
  m.m[0][1] = r.y;
  m.m[0][2] = r.z;
  m.m[0][3] = -v3_dot(r, eye);
  m.m[1][0] = u.x;
  m.m[1][1] = u.y;
  m.m[1][2] = u.z;
  m.m[1][3] = -v3_dot(u, eye);
  m.m[2][0] = -f.x;
  m.m[2][1] = -f.y;
  m.m[2][2] = -f.z;
  m.m[2][3] = v3_dot(f, eye);
  return m;
}

/*
 * m4_normal_mat — cofactor of the upper-left 3×3.
 *
 * Correctly transforms normals under non-uniform scale.
 * For a uniform-scale model (like our cube) this equals the rotation
 * part of the model matrix — but computing it properly means non-
 * uniform scale "just works" later if you change the model.
 */
static Mat4 m4_normal_mat(Mat4 m) {
  Mat4 n = m4_identity();
  n.m[0][0] = m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1];
  n.m[0][1] = m.m[1][2] * m.m[2][0] - m.m[1][0] * m.m[2][2];
  n.m[0][2] = m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0];
  n.m[1][0] = m.m[0][2] * m.m[2][1] - m.m[0][1] * m.m[2][2];
  n.m[1][1] = m.m[0][0] * m.m[2][2] - m.m[0][2] * m.m[2][0];
  n.m[1][2] = m.m[0][1] * m.m[2][0] - m.m[0][0] * m.m[2][1];
  n.m[2][0] = m.m[0][1] * m.m[1][2] - m.m[0][2] * m.m[1][1];
  n.m[2][1] = m.m[0][2] * m.m[1][0] - m.m[0][0] * m.m[1][2];
  n.m[2][2] = m.m[0][0] * m.m[1][1] - m.m[0][1] * m.m[1][0];
  return n;
}

/* ── §4 paint (216 RGB cube + Bourke ramp) ───────────────────────────── */

static int g_256;

/*
 * color_init — allocate 216 ncurses pairs as a 6×6×6 RGB cube + reserve
 * yellow PAIR_HUD and cyan PAIR_HINT for the HUD spec.
 *
 * Pair index = PAIR_CUBE_BASE + r·36 + g·6 + b   (r,g,b ∈ {0..5})
 *
 * Same scheme as the entire raytracer folder; reading any raytrace
 * file's color_init covers this one too.
 */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  if (g_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/*
 * Reinhard tone-map and gamma encode (matching path_tracer / raytracers).
 *   Reinhard: L' = L / (1 + L)   compresses HDR into [0, 1)
 *   Gamma:    out = L'^(1/2.2)   approximate sRGB transfer
 *
 * Without these, quantising to the 6×6×6 cube collapses every bright
 * cell into one cube cell because the cube is uniform in [0,1]³.
 */
static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* Cell stored in the framebuffer; blitted to ncurses at frame end. */
typedef struct {
  char ch;
  int pair;
  int attr;
} Cell;

/*
 * paint_compute_cell — full RGB → cell pipeline.
 *
 *   1. Reinhard tone-map per channel.
 *   2. Gamma encode 1/2.2.
 *   3. Bayer 4×4 dither on the luma so neighbouring cells get
 *      slightly different ramp characters even when their luma is
 *      identical (smoother gradient on a coarse grid).
 *   4. Quantise to 6×6×6 cube → pair id.
 *   5. Compute Rec.709 luma → density glyph from 92-char Bourke ramp.
 *   6. A_BOLD on the brightest bin, A_DIM on the darkest.
 */
static Cell paint_compute_cell(Vec3 col, int px, int py) {
  float r = gamma_enc(reinhard(col.x));
  float g = gamma_enc(reinhard(col.y));
  float b = gamma_enc(reinhard(col.z));

  /* Rec.709 luma (Bourke ramp expects perceptual brightness). */
  float luma = 0.2126f * r + 0.7152f * g + 0.0722f * b;

  /* Bayer dither perturbs the ramp index without changing the cube. */
  float dith = (k_bayer[py & 3][px & 3] - 0.5f) * DITHER_AMP;
  float lum_d = clamp01(luma + dith);

  Cell c;
  if (g_256) {
    int r5 = (int)(r * 5.f + 0.5f);
    if (r5 > 5)
      r5 = 5;
    if (r5 < 0)
      r5 = 0;
    int g5 = (int)(g * 5.f + 0.5f);
    if (g5 > 5)
      g5 = 5;
    if (g5 < 0)
      g5 = 0;
    int b5 = (int)(b * 5.f + 0.5f);
    if (b5 > 5)
      b5 = 5;
    if (b5 < 0)
      b5 = 0;
    c.pair = PAIR_CUBE_BASE + r5 * 36 + g5 * 6 + b5;
  } else {
    c.pair = PAIR_CUBE_BASE;
  }

  int idx = (int)(lum_d * (BOURKE_LEN - 1) + 0.5f);
  if (idx < 0)
    idx = 0;
  if (idx >= BOURKE_LEN)
    idx = BOURKE_LEN - 1;
  c.ch = k_bourke[idx];

  c.attr = (luma > 0.85f) ? A_BOLD : (luma < 0.15f) ? A_DIM : A_NORMAL;
  return c;
}

/* ── §5 shaders ──────────────────────────────────────────────────────── */

/* §5.1 ── shader interface types ─────────────────────────────────────── */

typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} VSIn;

typedef struct {
  Vec4 clip_pos;
  Vec3 world_pos;
  Vec3 world_nrm;
  float u, v;
  float custom[4]; /* generic per-vertex output (e.g. baryco) */
} VSOut;

typedef struct {
  Vec3 world_pos;
  Vec3 world_nrm;
  float u, v;
  float custom[4];
  int px, py;
} FSIn;

typedef struct {
  Vec3 color;
  bool discard;
} FSOut;

typedef void (*VertShaderFn)(const VSIn *in, VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in, FSOut *out, const void *uni);

typedef struct {
  VertShaderFn vert;
  FragShaderFn frag;
  const void *vert_uni;
  const void *frag_uni;
} ShaderProgram;

/* Uniforms — global per-frame state shared by all shaders. */
typedef struct {
  Mat4 model, view, proj, mvp, norm_mat;
  Vec3 light_pos, light_col, ambient, cam_pos, obj_color;
  float shininess;
} Uniforms;

/* Toon shader needs an extra "bands" parameter. */
typedef struct {
  Uniforms base;
  int bands;
} ToonUniforms;

/* §5.2 ── default vertex shader ──────────────────────────────────────── */

/*
 * Almost every fragment shader needs the same per-vertex computations:
 * world position, world-space normal, MVP-transformed clip position.
 * The default vertex shader does ALL of those; specialised shaders
 * (vert_normals, vert_wire) extend it by writing extra `custom`
 * outputs.
 */
static void vert_default(const VSIn *in, VSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  out->clip_pos = m4_mul_v4(u->mvp, v4(in->pos.x, in->pos.y, in->pos.z, 1.f));
  out->world_pos = m4_pt(u->model, in->pos);
  out->world_nrm = v3_norm(m4_dir(u->norm_mat, in->normal));
  out->u = in->u;
  out->v = in->v;
  out->custom[0] = out->custom[1] = out->custom[2] = out->custom[3] = 0.f;
}

/*
 * vert_normals — same as default + writes the world normal into custom[]
 * so the fragment shader can read it directly without recomputing.
 * (frag_normals doesn't actually use custom, but this is a useful
 * pattern shown for the wire shader below.)
 */
static void vert_normals(const VSIn *in, VSOut *out, const void *u_) {
  vert_default(in, out, u_);
  out->custom[0] = out->world_nrm.x;
  out->custom[1] = out->world_nrm.y;
  out->custom[2] = out->world_nrm.z;
}

/*
 * vert_wire — barycentric-coord injection.
 *
 * The pipeline (§8.2) sets VSIn.u/v to the per-vertex BARYCENTRIC
 * IDENTITY VECTOR (1,0,0)/(0,1,0)/(0,0,1) BEFORE this shader runs.
 * We just call the default; the pipeline writes custom[0..2] AFTER.
 *
 * Inside the rasteriser, custom[0..2] are interpolated by barycentric
 * weights (b0, b1, b2). When all three are large simultaneously, we're
 * deep inside the triangle; when any one is near zero, we're at an
 * edge. min(custom[0..2]) is the distance to the nearest edge.
 */
static void vert_wire(const VSIn *in, VSOut *out, const void *u_) {
  vert_default(in, out, u_);
  /* custom[0..2] zeroed by vert_default; pipeline overwrites them. */
}

/* §5.3 ── frag_phong: Blinn-Phong shading ───────────────────────────── */

/*
 * Blinn-Phong = ambient + diffuse + specular.
 *   diffuse = max(0, N·L)                 Lambertian
 *   spec    = max(0, N·H)^shininess       half-vector form
 *   col     = ambient + obj·light·diff + spec·0.5
 *   gamma   = col^(1/2.2)
 *
 * On the cube this is easy to verify because each face has a uniform
 * normal: every fragment on a face sees the same N, L, H → uniform
 * diffuse + a single specular highlight per face.
 */
static void frag_phong(const FSIn *in, FSOut *out, const void *u_) {
  const Uniforms *u = (const Uniforms *)u_;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  float diff = fmaxf(0.f, v3_dot(N, L));
  float spec = powf(fmaxf(0.f, v3_dot(N, H)), u->shininess);

  Vec3 c = u->obj_color;
  out->color = v3(u->ambient.x + c.x * u->light_col.x * diff + spec * 0.5f,
                  u->ambient.y + c.y * u->light_col.y * diff + spec * 0.5f,
                  u->ambient.z + c.z * u->light_col.z * diff + spec * 0.5f);
  out->discard = false;
}

/* §5.4 ── frag_toon: 4-band quantised diffuse + hard specular ───────── */

/*
 * Quantising the diffuse term to discrete bands gives the cel-shaded
 * look: instead of a smooth gradient, you see hard steps. For a cube,
 * each face is uniformly lit so each face is entirely one band — the
 * silhouette between a bright and a dim face is a perfectly sharp
 * boundary.
 *
 * The hard specular threshold (N·H > 0.94) gives the cel highlight:
 * either ON (bright spot) or OFF — no falloff.
 */
static void frag_toon(const FSIn *in, FSOut *out, const void *u_) {
  const ToonUniforms *tu = (const ToonUniforms *)u_;
  const Uniforms *u = &tu->base;
  Vec3 N = v3_norm(in->world_nrm);
  Vec3 L = v3_norm(v3_sub(u->light_pos, in->world_pos));
  Vec3 V = v3_norm(v3_sub(u->cam_pos, in->world_pos));
  Vec3 H = v3_norm(v3_add(L, V));

  float diff = fmaxf(0.f, v3_dot(N, L));
  float banded = floorf(diff * (float)tu->bands) / (float)tu->bands;
  float spec = (v3_dot(N, H) > 0.94f) ? 0.7f : 0.f;

  Vec3 c = u->obj_color;
  out->color = v3(c.x * (banded + 0.12f) + spec, c.y * (banded + 0.12f) + spec,
                  c.z * (banded + 0.12f) + spec);
  out->discard = false;
}

/* §5.5 ── frag_normals: world-normal-as-RGB (diagnostic) ─────────────── */

/*
 * Each component remapped from [-1, +1] → [0, 1]. For a cube, each
 * face has a uniform normal so each face renders as ONE solid colour:
 *   +X → (1.0, 0.5, 0.5)   -X → (0.0, 0.5, 0.5)
 *   +Y → (0.5, 1.0, 0.5)   -Y → (0.5, 0.0, 0.5)
 *   +Z → (0.5, 0.5, 1.0)   -Z → (0.5, 0.5, 0.0)
 *
 * The colours rotate as the cube rotates (world-space normal).
 */
static void frag_normals(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  Vec3 N = v3_norm(in->world_nrm);
  out->color = v3(N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f);
  out->discard = false;
}

/* §5.6 ── frag_wire: barycentric edge detection ─────────────────────── */

/*
 * `min(custom[0..2])` is the distance to the nearest TRIANGLE edge
 * (the pipeline interpolates the barycentric identity vector across
 * fragments, and the smallest of (b0, b1, b2) is the distance to the
 * opposite edge of the triangle we're inside).
 *
 * Below WIRE_THRESH: draw the edge with a brightness proportional to
 * "how close to the edge we are" (sharper near the line itself).
 * Above WIRE_THRESH: discard — the triangle interior is transparent.
 */
static void frag_wire(const FSIn *in, FSOut *out, const void *u_) {
  (void)u_;
  float b0 = in->custom[0], b1 = in->custom[1], b2 = in->custom[2];
  float edge = fminf(b0, fminf(b1, b2));
  if (edge > WIRE_THRESH) {
    out->discard = true;
    return;
  }
  float t = edge / WIRE_THRESH;
  out->color = v3(0.9f - t * 0.3f, 0.9f - t * 0.3f, 0.9f - t * 0.3f);
  out->discard = false;
}

typedef enum { SH_PHONG = 0, SH_TOON, SH_NORMALS, SH_WIRE, SH_COUNT } ShaderIdx;
static const char *k_shader_names[] = {"phong", "toon", "normals", "wire"};

/* ── §6 mesh — cube tessellation ─────────────────────────────────────── */

typedef struct {
  Vec3 pos;
  Vec3 normal;
  float u, v;
} Vertex;
typedef struct {
  int v[3];
} Triangle;
typedef struct {
  Vertex *verts;
  int nvert;
  Triangle *tris;
  int ntri;
} Mesh;

static void mesh_free(Mesh *m) {
  free(m->verts);
  free(m->tris);
  *m = (Mesh){0};
}

/*
 * tessellate_cube — 6 faces, each a quad split into 2 triangles.
 *
 * Each face has 4 unique vertices with the SAME outward normal — flat
 * shading, no shared vertices between faces. Sharing vertices would
 * average normals at corners and round the cube (correct for a sphere,
 * wrong for a hard-surface cube).
 *
 * Winding order: CCW when viewed from outside (from the direction the
 * normal points). The pipeline culls CW back faces (signed-area test
 * after Y-flip → CCW maps to positive area).
 *
 * UV layout per face: each face maps [0,1]² independently.
 *   v0 = bottom-left  (0,1)
 *   v1 = bottom-right (1,1)
 *   v2 = top-right    (1,0)
 *   v3 = top-left     (0,0)
 *
 * Total: 24 vertices, 12 triangles. Mesh allocated ONCE at startup.
 */
static Mesh tessellate_cube(void) {
  float s = CUBE_S;

  static const float face_nrm[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
  };

  /* Corners per face in CCW winding viewed from outside. Each corner
   * is in -1..+1 and gets scaled by s in the build loop. */
  static const float face_vtx[6][4][3] = {
      /* +X */ {{1, -1, 1}, {1, 1, 1}, {1, 1, -1}, {1, -1, -1}},
      /* -X */ {{-1, -1, -1}, {-1, 1, -1}, {-1, 1, 1}, {-1, -1, 1}},
      /* +Y */ {{-1, 1, 1}, {1, 1, 1}, {1, 1, -1}, {-1, 1, -1}},
      /* -Y */ {{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}},
      /* +Z */ {{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}},
      /* -Z */ {{1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}},
  };

  static const float face_uv[4][2] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};

  Mesh m;
  m.verts = malloc(24 * sizeof(Vertex));
  m.tris = malloc(12 * sizeof(Triangle));
  m.nvert = 0;
  m.ntri = 0;

  for (int f = 0; f < 6; f++) {
    Vec3 n = v3(face_nrm[f][0], face_nrm[f][1], face_nrm[f][2]);
    int base = m.nvert;

    for (int i = 0; i < 4; i++) {
      Vec3 p = v3(face_vtx[f][i][0] * s, face_vtx[f][i][1] * s,
                  face_vtx[f][i][2] * s);
      m.verts[m.nvert++] = (Vertex){p, n, face_uv[i][0], face_uv[i][1]};
    }

    /* Split quad into 2 CCW triangles:
     *   tri 0: v0, v1, v2  (bottom-left → bottom-right → top-right)
     *   tri 1: v0, v2, v3  (bottom-left → top-right → top-left) */
    m.tris[m.ntri++] = (Triangle){{base + 0, base + 1, base + 2}};
    m.tris[m.ntri++] = (Triangle){{base + 0, base + 2, base + 3}};
  }

  return m;
}

/* ── §7 framebuffer ──────────────────────────────────────────────────── */

typedef struct {
  float *zbuf;
  Cell *cbuf;
  int cols, rows;
} Framebuffer;

static void fb_alloc(Framebuffer *fb, int cols, int rows) {
  fb->cols = cols;
  fb->rows = rows;
  fb->zbuf = malloc((size_t)(cols * rows) * sizeof(float));
  fb->cbuf = malloc((size_t)(cols * rows) * sizeof(Cell));
}

static void fb_free(Framebuffer *fb) {
  free(fb->zbuf);
  free(fb->cbuf);
  *fb = (Framebuffer){0};
}

static void fb_clear(Framebuffer *fb) {
  for (int i = 0; i < fb->cols * fb->rows; i++)
    fb->zbuf[i] = FLT_MAX;
  memset(fb->cbuf, 0, (size_t)(fb->cols * fb->rows) * sizeof(Cell));
}

/* fb_blit — copy framebuffer cbuf to ncurses screen. */
static void fb_blit(const Framebuffer *fb) {
  for (int y = 0; y < fb->rows; y++) {
    for (int x = 0; x < fb->cols; x++) {
      Cell c = fb->cbuf[y * fb->cols + x];
      if (!c.ch)
        continue;
      attron(COLOR_PAIR(c.pair) | c.attr);
      mvaddch(y, x, (chtype)(unsigned char)c.ch);
      attroff(COLOR_PAIR(c.pair) | c.attr);
    }
  }
}

/* ── §8 pipeline ─────────────────────────────────────────────────────── */

/* §8.1 ── barycentric (Möller signed-area form) ─────────────────────── */

/*
 * Compute barycentric coordinates of point (px, py) inside triangle
 * with vertices (sx[0..2], sy[0..2]) in screen space.
 *
 * Outputs b[0..2] such that:
 *   - b[0] + b[1] + b[2] = 1
 *   - All three positive iff (px, py) is strictly inside the triangle
 *   - Each b[i] is the SIGNED RATIO of "sub-triangle area opposite
 *     vertex i" to "full triangle area"
 *
 * If the triangle is degenerate (area ≈ 0), returns negative values
 * so the caller treats it as a miss.
 */
static void barycentric(const float sx[3], const float sy[3], float px,
                        float py, float b[3]) {
  float d =
      (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
  if (fabsf(d) < 1e-6f) {
    b[0] = b[1] = b[2] = -1.f;
    return;
  }
  b[0] = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / d;
  b[1] = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / d;
  b[2] = 1.f - b[0] - b[1];
}

/* §8.2 ── pipeline_draw_mesh — vertex → cull → raster → fragment ────── */

/*
 * The big function. For each triangle in the mesh:
 *   1. Run the vertex shader on its 3 vertices.
 *   2. Reject if all 3 are behind the near plane (w < ε).
 *   3. Perspective-divide to NDC, map to screen.
 *   4. Back-face cull (signed-area test).
 *   5. Walk the screen-space bounding box.
 *   6. For each pixel: barycentric check + z-test + fragment shader
 *      + paint_compute_cell + framebuffer write.
 *
 * The `is_wire` flag inserts barycentric-identity values into VSIn.u/v
 * BEFORE the vertex shader, then writes them to VSOut.custom[0..2]
 * AFTER. Inside the rasteriser, custom[0..2] are interpolated by the
 * barycentric weights. The fragment shader (frag_wire) reads those
 * three to compute distance-to-nearest-edge.
 */
static void pipeline_draw_mesh(Framebuffer *fb, const Mesh *mesh,
                               ShaderProgram *sh, bool is_wire,
                               bool cull_backface) {
  int cols = fb->cols, rows = fb->rows;
  static const float wu[3] = {1.f, 0.f, 0.f};
  static const float wv[3] = {0.f, 1.f, 0.f};

  for (int ti = 0; ti < mesh->ntri; ti++) {
    const Triangle *tri = &mesh->tris[ti];
    VSOut vo[3];

    /* (1) Run vertex shader on each of the triangle's 3 vertices. */
    for (int vi = 0; vi < 3; vi++) {
      const Vertex *vtx = &mesh->verts[tri->v[vi]];
      VSIn in;
      in.pos = vtx->pos;
      in.normal = vtx->normal;
      in.u = is_wire ? wu[vi] : vtx->u;
      in.v = is_wire ? wv[vi] : vtx->v;
      memset(&vo[vi], 0, sizeof vo[vi]);
      sh->vert(&in, &vo[vi], sh->vert_uni);
      if (is_wire) {
        vo[vi].custom[0] = wu[vi];
        vo[vi].custom[1] = wv[vi];
        vo[vi].custom[2] = 1.f - wu[vi] - wv[vi];
      }
    }

    /* (2) Near-clip reject — entirely behind camera. */
    if (vo[0].clip_pos.w < 0.001f && vo[1].clip_pos.w < 0.001f &&
        vo[2].clip_pos.w < 0.001f)
      continue;

    /* (3) Perspective divide → screen coordinates.
     * Y-flip because NDC has +Y up but screen rows go down. */
    float sx[3], sy[3], sz[3];
    for (int vi = 0; vi < 3; vi++) {
      float w = vo[vi].clip_pos.w;
      if (fabsf(w) < 1e-6f)
        w = 1e-6f;
      sx[vi] = (vo[vi].clip_pos.x / w + 1.f) * 0.5f * (float)cols;
      sy[vi] = (-vo[vi].clip_pos.y / w + 1.f) * 0.5f * (float)rows;
      sz[vi] = vo[vi].clip_pos.z / w;
    }

    /* (4) Back-face cull (after Y-flip, CCW = positive area). */
    float area =
        (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
    if (cull_backface && area <= 0.f)
      continue;

    /* (5) Screen-space bounding box (clamped to framebuffer). */
    int x0 = (int)fmaxf(0.f, floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = (int)fminf(cols - 1.f, ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = (int)fmaxf(0.f, floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = (int)fminf(rows - 1.f, ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));

    /* (6) Walk every pixel in the bbox; rasterise. */
    for (int py = y0; py <= y1; py++) {
      for (int px = x0; px <= x1; px++) {
        float b[3];
        barycentric(sx, sy, px + 0.5f, py + 0.5f, b);
        if (b[0] < 0.f || b[1] < 0.f || b[2] < 0.f)
          continue;

        /* Z-test (linear interpolation of NDC depth). */
        float z = b[0] * sz[0] + b[1] * sz[1] + b[2] * sz[2];
        int idx = py * cols + px;
        if (z >= fb->zbuf[idx])
          continue;
        fb->zbuf[idx] = z;

        /* Build fragment-shader input by barycentric interpolation. */
        FSIn fsin;
        fsin.world_pos = v3_bary(vo[0].world_pos, vo[1].world_pos,
                                 vo[2].world_pos, b[0], b[1], b[2]);
        fsin.world_nrm = v3_norm(v3_bary(vo[0].world_nrm, vo[1].world_nrm,
                                         vo[2].world_nrm, b[0], b[1], b[2]));
        fsin.u = b[0] * vo[0].u + b[1] * vo[1].u + b[2] * vo[2].u;
        fsin.v = b[0] * vo[0].v + b[1] * vo[1].v + b[2] * vo[2].v;
        fsin.px = px;
        fsin.py = py;
        for (int c = 0; c < 4; c++)
          fsin.custom[c] = b[0] * vo[0].custom[c] + b[1] * vo[1].custom[c] +
                           b[2] * vo[2].custom[c];

        FSOut fsout = {v3(0, 0, 0), false};
        sh->frag(&fsin, &fsout, sh->frag_uni);
        if (fsout.discard)
          continue;

        fb->cbuf[idx] = paint_compute_cell(fsout.color, px, py);
      }
    }
  }
}

/* ── §9 scene ────────────────────────────────────────────────────────── */

typedef struct {
  Mesh mesh;
  float angle_x, angle_y;
  float cam_dist;
  bool paused;
  bool cull_backface;
  ShaderIdx shade_idx;
  ShaderProgram shader;
  Uniforms uni;
  ToonUniforms toon_uni;
} Scene;

static void scene_build_shader(Scene *s) {
  switch (s->shade_idx) {
  case SH_PHONG:
    s->shader = (ShaderProgram){vert_default, frag_phong, &s->uni, &s->uni};
    break;
  case SH_TOON:
    s->toon_uni.base = s->uni;
    s->toon_uni.bands = 4;
    s->shader = (ShaderProgram){vert_default, frag_toon, &s->uni, &s->toon_uni};
    break;
  case SH_NORMALS:
    s->shader = (ShaderProgram){vert_normals, frag_normals, &s->uni, &s->uni};
    break;
  case SH_WIRE:
    s->shader = (ShaderProgram){vert_wire, frag_wire, &s->uni, &s->uni};
    break;
  default:
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->mesh = tessellate_cube();
  s->shade_idx = SH_PHONG;
  s->cam_dist = CAM_DIST;
  s->cull_backface = true;

  s->uni.light_pos = v3(4.f, 5.f, 4.f);
  s->uni.light_col = v3(1.f, 1.f, 1.f);
  s->uni.ambient = v3(0.07f, 0.07f, 0.07f);
  s->uni.shininess = 64.f;
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.obj_color = v3(0.9f, 0.55f, 0.15f); /* warm orange */

  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);

  scene_build_shader(s);
}

/*
 * scene_set_zoom — update camera position and view matrix.
 * Called on +/- keys; rebuilds cam_pos and view so the change takes
 * effect on the very next scene_tick → mvp rebuild.
 */
static void scene_set_zoom(Scene *s) {
  s->uni.cam_pos = v3(0.f, 0.f, s->cam_dist);
  s->uni.view = m4_lookat(s->uni.cam_pos, v3(0, 0, 0), v3(0, 1, 0));
}

static void scene_rebuild_proj(Scene *s, int cols, int rows) {
  float aspect = (float)(cols * CELL_W) / (float)(rows * CELL_H);
  s->uni.proj = m4_perspective(CAM_FOV, aspect, CAM_NEAR, CAM_FAR);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->angle_y += ROT_Y * dt;
  s->angle_x += ROT_X * dt;
  Mat4 ry = m4_rotate_y(s->angle_y);
  Mat4 rx = m4_rotate_x(s->angle_x);
  s->uni.model = m4_mul(ry, rx);
  s->uni.mvp = m4_mul(s->uni.proj, m4_mul(s->uni.view, s->uni.model));
  s->uni.norm_mat = m4_normal_mat(s->uni.model);
  s->toon_uni.base = s->uni;
}

static void scene_draw(Scene *s, Framebuffer *fb) {
  fb_clear(fb);
  pipeline_draw_mesh(fb, &s->mesh, &s->shader, (s->shade_idx == SH_WIRE),
                     s->cull_backface);
  fb_blit(fb);
}

static void scene_next_shader(Scene *s) {
  s->shade_idx = (ShaderIdx)((s->shade_idx + 1) % SH_COUNT);
  scene_build_shader(s);
}

/* ── §10 screen ──────────────────────────────────────────────────────── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * hud_draw — yellow status row 0 + cyan hint bottom row.
 * Per CLAUDE.md HUD spec: PAIR_HUD is bright yellow (xterm 226) with
 * A_BOLD; PAIR_HINT is bright cyan (xterm 51) with A_BOLD.
 */
static void hud_draw(const Screen *s, const Scene *sc, double fps) {
  char buf[96];
  snprintf(buf, sizeof buf, " %5.1f fps  [%s]  z:%.1f  cull:%s%s ", fps,
           k_shader_names[sc->shade_idx], (double)sc->cam_dist,
           sc->cull_backface ? "on " : "off", sc->paused ? " PAUSED" : "");
  int len = (int)strlen(buf);
  if (len > s->cols)
    len = s->cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " CUBE-RASTER ");
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0, " q:quit  spc:pause  s:shader  c:cull  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §11 app ─────────────────────────────────────────────────────────── */

typedef struct {
  Scene scene;
  Screen screen;
  Framebuffer fb;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  fb_free(&app->fb);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  scene_rebuild_proj(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 's':
  case 'S':
    scene_next_shader(s);
    break;
  case 'c':
  case 'C':
    s->cull_backface = !s->cull_backface;
    break;
  case '=':
  case '+':
    s->cam_dist -= CAM_ZOOM_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    scene_set_zoom(s);
    break;
  case '-':
  case '_':
    s->cam_dist += CAM_ZOOM_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    scene_set_zoom(s);
    break;
  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)clock_ns());
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;

  screen_init(&app->screen);
  fb_alloc(&app->fb, app->screen.cols, app->screen.rows);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  double fps_disp = 0.0;

  while (app->running) {

    /* §11.1 resize. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
    }

    /* §11.2 timing. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    float dt_sec = (float)dt / (float)NS_PER_SEC;

    /* §11.3 advance scene state. */
    scene_tick(&app->scene, dt_sec);

    /* §11.4 fps rolling average over half-second windows. */
    fps_cnt++;
    fps_acc += dt;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_cnt / ((double)fps_acc / (double)NS_PER_SEC);
      fps_cnt = 0;
      fps_acc = 0;
    }

    /* §11.5 paint frame. */
    erase();
    scene_draw(&app->scene, &app->fb);
    hud_draw(&app->screen, &app->scene, fps_disp);
    screen_present();

    /* §11.6 input. */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;

    /* §11.7 frame cap. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / FPS_TARGET - elapsed);
  }

  mesh_free(&app->scene.mesh);
  fb_free(&app->fb);
  endwin();
  return 0;
}
