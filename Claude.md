# Terminal Demos — ncurses C Projects

## Build
```bash
gcc -std=c11 -O2 -Wall -Wextra bounce.c      -o bounce      -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra matrix_rain.c -o matrix_rain -lncurses -lm

# raster demos
gcc -std=c11 -O2 -Wall -Wextra raster/torus_raster.c    -o torus    -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raster/cube_raster.c     -o cube     -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raster/sphere_raster.c   -o sphere   -lncurses -lm
gcc -std=c11 -O2 -Wall -Wextra raster/displace_raster.c -o displace -lncurses -lm
```

## Files
- `bounce.c`      — bouncing balls, smooth terminal-aware physics
- `matrix_rain.c` — Matrix-style falling character rain

### raster/
- `torus_raster.c`    — UV torus, 4 shaders (phong/toon/normals/wireframe)
- `cube_raster.c`     — unit cube with flat normals, same 4 shaders + zoom/cull toggle
- `sphere_raster.c`   — UV sphere, same 4 shaders + zoom/cull toggle
- `displace_raster.c` — UV sphere with real-time vertex displacement (ripple/wave/pulse/spiky)

---

## Core Architecture (both files)

### Coordinate / Physics Model (bounce.c)
- Physics lives entirely in **pixel space** — one unit = one physical pixel
- Terminal cells are ~2× taller than wide; pixel space compensates:
  - `CELL_W = 8`, `CELL_H = 16`
  - pixel width  = `cols * CELL_W`
  - pixel height = `rows * CELL_H`
- Ball position (`px`, `py`) and velocity (`vx`, `vy`) are always floats in pixel space
- **One conversion point only**: `px_to_cell_x/y()` in `scene_draw()` — nowhere else
- `floorf(px / CELL_W + 0.5f)` used for rounding — "round half up", not `roundf()`
  (avoids banker's rounding oscillation at cell boundaries)

### Simulation Loop (both files)
- Fixed-timestep accumulator pattern:
  ```c
  sim_accum += dt;
  while (sim_accum >= tick_ns) {
      scene_tick / rain_tick(...);
      sim_accum -= tick_ns;
  }
  ```
- `dt` clamped to 100 ms to prevent spiral-of-death after stalls
- Render frame cap: sleep to target 60 fps render rate
- `sim_fps` and render fps are independent — sim can run at 20 Hz while rendering at 60 Hz

### Render Interpolation — alpha (both files)
- After draining the accumulator, `sim_accum` = leftover ns into the next unfired tick
- `alpha = (float)sim_accum / (float)tick_ns`  ∈ [0.0, 1.0)
- **bounce.c**: draw position projected forward:
  ```c
  draw_px = b->px + b->vx * alpha * dt_sec;
  draw_py = b->py + b->vy * alpha * dt_sec;
  ```
- **matrix_rain.c**: column head projected forward:
  ```c
  draw_head_y = (float)c->head_y + (float)c->speed * alpha;
  ```
  then `row = (int)floorf(draw_head_y - dist + 0.5f)`
- Physics state (`px`, `py`, `head_y`) is **never modified** by interpolation — draw only
- Forward extrapolation is correct for constant-velocity / integer-speed motion;
  if acceleration is added, switch to storing `prev_pos` and lerping between prev and current

### ncurses Rendering (both files)
- **Single `stdscr` buffer** — no back/front WINDOW pair
- ncurses internally maintains `curscr` (current) and `newscr` (target)
- Frame sequence every render:
  ```
  erase()              — clear newscr
  draw balls/rain      — write into newscr
  draw HUD             — written last, always on top
  wnoutrefresh(stdscr) — mark newscr ready, no terminal I/O yet
  doupdate()           — one atomic diff write to terminal fd
  ```
- `typeahead(-1)` — prevents ncurses interrupting mid-flush to poll stdin
- Adding a second WINDOW breaks ncurses' diff accuracy → ghost trails

### Signal Handling (both files)
- `SIGINT` / `SIGTERM` → set `running = 0` (clean exit)
- `SIGWINCH` → set `need_resize = 1` (handled at top of main loop)
- `atexit(cleanup)` calls `endwin()` — terminal always restored even on crash
- Resize resets `frame_time` and `sim_accum` to avoid dt spike after rebuild stall

---

## matrix_rain.c Specifics

### Two-pass Draw (interpolated render)
- **Pass 1 — grid**: persistent dissolve/fade texture from `rain_tick()`; integer positions
- **Pass 2 — live columns**: drawn directly to `stdscr` via `col_paint_interpolated()`
  at float `draw_head_y`; bypasses the grid entirely
- Grid still exists for `grid_scatter_erase()` — stochastic per-row erasure gives
  the organic fade-to-black look; without it columns vanish instantly

### Column
- `head_y` is integer physics position; only the draw function uses float projection
- `ch_cache[TRAIL_MAX]` — characters seeded at spawn, refreshed each `col_tick()`
  so `col_paint_interpolated()` always has glyphs without consulting the grid
- Shade gradient by `dist` from head: HEAD → HOT → BRIGHT → MID → DARK → FADE

### Themes
- 4 themes: green / amber / blue / white
- 256-color: distinct xterm-256 index per shade level (rich gradient)
- 8-color fallback: same base color, `A_DIM` / `A_BOLD` carry the gradient
- `theme_apply()` selects palette at runtime via `COLORS >= 256` check

---

## bounce.c Specifics

### Speed / Smoothness Rule
- `SPEED_MIN = 300 px/s`, `SPEED_MAX = 600 px/s`
- Minimum speed formula: `SPEED_MIN >= CELL_H * SIM_FPS / 4 = 240`
  (ensures balls cross cell boundaries often enough to avoid staircase)
- Velocity direction: rejection-sample unit vector → isotropic angle distribution
  (separate random `vx`/`vy` skews toward diagonal — don't do that)

### Wall Bounce
- Clamp position to `[0, max_px]` / `[0, max_py]` and flip velocity component
- `max_px = pw(cols) - 1`, `max_py = ph(rows) - 1`

---

---

## raster/*.c — Software Rasterizer

### Pipeline
Each raster file is a self-contained software renderer:
```
tessellate_*()  →  scene_tick()  →  pipeline_draw_mesh()  →  fb_blit()
                   (rotate MVP)      for each triangle:
                                       vert shader  (VSIn → VSOut)
                                       clip/NDC/screen
                                       back-face cull
                                       rasterize (barycentric)
                                       z-test
                                       frag shader  (FSIn → FSOut)
                                       luma → dither → Paul Bourke char → cbuf
```

### ShaderProgram — split vert_uni / frag_uni (segfault fix)
All four raster files use:
```c
typedef struct {
    VertShaderFn  vert;
    FragShaderFn  frag;
    const void   *vert_uni;   /* passed to vert() */
    const void   *frag_uni;   /* passed to frag() */
} ShaderProgram;
```
**Why split:** the vertex and fragment shaders can require different uniform struct types.
In `displace_raster.c`, `vert_displace` needs `DisplaceUniforms` (has `disp_fn`, `time`,
`amplitude`, `frequency`) while `frag_toon` needs `ToonUniforms` (has `bands`). A single
`void *uniforms` pointer cannot satisfy both — whichever shader receives the wrong type
will cast it and read garbage, causing a segfault when it dereferences a null function
pointer or out-of-range field.

The split was applied to **all four raster files** for consistency:

| Shader | vert_uni | frag_uni |
|---|---|---|
| phong   | `&s->uni`      | `&s->uni`      |
| toon    | `&s->uni`      | `&s->toon_uni` |
| normals | `&s->uni`      | `&s->uni`      |
| wire    | `&s->uni`      | `&s->uni`      |

For toon: `vert_uni = &s->uni` (vert shader only needs `Uniforms`); `frag_uni = &s->toon_uni`
(`frag_toon` needs `ToonUniforms.bands`). `ToonUniforms` leads with `Uniforms base` so the
vert shader's `(const Uniforms*)vert_uni` cast is safe (zero-offset rule).

### Framebuffer
- `zbuf[cols*rows]` — float depth buffer, cleared to `FLT_MAX`
- `cbuf[cols*rows]` — `Cell{ch, color_pair, bold}`, blitted to stdscr each frame
- `luma_to_cell(luma, px, py)` — Bayer 4×4 ordered dither → Paul Bourke ASCII ramp + 7-color palette

### Displacement (displace_raster.c only)
- 4 modes (ripple/wave/pulse/spiky) — each a pure `float fn(Vec3, time, amp, freq)`
- Normal recomputation via central difference: sample displacement at `pos ± eps*T` and `pos ± eps*B`, reconstruct displaced tangent vectors, take cross product
- `DisplaceUniforms` extends `Uniforms` with `disp_fn`, `time`, `amplitude`, `frequency`, `mode`

---

## Coding Conventions
- Comments explain **why**, not just what — especially non-obvious physics/rounding choices
- Every tunable constant lives in `§1 config` as an enum or `#define`
- Functions are small and single-purpose; data flows in one direction
- No dynamic allocation after init except `rain_init()` column/grid arrays
- `sig_atomic_t` for all signal-written flags
- C11, `-Wall -Wextra` clean
