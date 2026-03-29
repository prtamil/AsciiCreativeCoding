# NCURSES_TECHNIQUES.md — Per-File ncurses Technique Reference

Each section lists only the techniques that are **specific or notable** for that file.
Cross-file universals (initscr, noecho, cbreak, curs_set, nodelay, keypad, atexit/endwin,
SIGINT/SIGWINCH, erase→wnoutrefresh→doupdate loop) are listed once in the preamble and
omitted from individual entries unless the file does something unusual with them.

---

## Universal Patterns (present in almost every animation file)

| Pattern | Code form |
|---|---|
| Init sequence | `initscr → noecho → cbreak → curs_set(0) → nodelay(stdscr,TRUE) → keypad(stdscr,TRUE) → typeahead(-1)` |
| Color guard | `if (has_colors()) { start_color(); … }` |
| 256-color fallback | `COLORS >= 256` branch → xterm-256 pairs; else `8`-color pairs |
| Atomic diff write | `typeahead(-1)` — disables mid-flush stdin poll |
| Frame sequence | `erase() → draw → wnoutrefresh(stdscr) → doupdate()` |
| Stable frame cap | `clock_sleep_ns()` **before** terminal I/O |
| Signal flags | `volatile sig_atomic_t running, need_resize` |
| Terminal restore | `atexit(cleanup)` → `endwin()` |
| SIGWINCH resize | `endwin() + refresh() + getmaxyx()` on flag |
| Safe mvaddch | `(chtype)(unsigned char)ch` double cast — prevents sign-extension |

---

## ncurses_basics/tst_lines_cols.c

**Purpose:** Minimal `LINES`/`COLS` demo.

| Technique | Detail |
|---|---|
| `LINES` / `COLS` globals | Read directly after `initscr()` — no `getmaxyx()` |
| `printw` / `refresh()` | Simplest possible output path — no scene loop, no color |
| Blocking `getch()` | Waits indefinitely for one keypress then exits |
| No signal handling | No `SIGWINCH`, no `sig_atomic_t` — purely illustrative |
| No `start_color()` | No color init whatsoever |

---

## ncurses_basics/aspect_ratio.c

**Purpose:** Correct-looking circle via x-coordinate doubling.

| Technique | Detail |
|---|---|
| `newwin(max_y, max_x, 0, 0)` | Manual WINDOW* — the pre-reference anti-pattern; later files all use `stdscr` |
| `wbkgd(win, COLOR_PAIR(1))` | Sets window background attribute (fills cleared cells) |
| `werase(win)` / `wrefresh(win)` | Per-window erase and single-window refresh (not `doupdate`) |
| `raw()` instead of `cbreak()` | Ctrl-C delivered as character 3 — **no SIGINT generated** |
| Aspect correction | `x = cx + radius * 2 * cos(angle)` — multiply x by 2 to compensate for tall cells |

---

## misc/bounce_ball.c

**Purpose:** Reference implementation — canonical ncurses animation skeleton.

| Technique | Detail |
|---|---|
| 7 xterm-256 color pairs | 196 red, 208 orange, 226 yellow, 46 green, 51 cyan, 21 blue, 201 magenta; 8-color fallback |
| `wattron / mvwaddch / wattroff` | Per-character attribute bracket — `wattron(w, COLOR_PAIR(b->color)|A_BOLD)` |
| `(chtype)(unsigned char)b->ch` | Double cast on `mvwaddch` — canonical safe form |
| HUD via `mvprintw` | `attron(COLOR_PAIR(3)|A_BOLD)` → `mvprintw(0, hud_x, "%s", buf)` → `attroff` |
| Pixel-space physics | `CELL_W=8`, `CELL_H=16` sub-pixels; `floorf(px/CELL_W + 0.5f)` round-half-up |
| Forward extrapolation | `draw_px = b->px + b->vx * alpha * dt_sec` — constant-velocity interpolation |
| `typeahead(-1)` | Explicitly documented in comments as the atomic-write trick |

---

## ncurses_basics/spring_pendulum.c

**Purpose:** Lagrangian spring-pendulum with Bresenham coil rendering.

| Technique | Detail |
|---|---|
| 5 named color pairs | `CP_BAR(231/white)`, `CP_WIRE(245/grey)`, `CP_SPRING(220/yellow)`, `CP_BALL(252/lgrey)`, `CP_HUD(51/cyan)` — semantic names, not indices |
| Slope char from Bresenham step | `(step_x && step_y) ? (sx==sy ? '\\' : '/') : step_x ? '-' : '|'` — one expression |
| Layered draw order | Bar → wire stubs → coil lines → coil nodes (overwrites lines) → bob (overwrites nodes) — later `mvaddch` wins |
| `prev_r`/`prev_theta` lerp | `draw_r = prev_r + (r - prev_r) * alpha` — non-linear forces use prev/cur lerp |

---

## matrix_rain/matrix_rain.c

**Purpose:** Matrix-style falling character rain.

| Technique | Detail |
|---|---|
| `use_default_colors()` + pair `-1` bg | Terminal background shows through — transparency |
| 6-shade `Shade` enum | `FADE(A_DIM)`, `DARK`, `MID`, `BRIGHT(A_BOLD)`, `HOT(A_BOLD)`, `HEAD(A_BOLD)` — maps to `shade_attr()` |
| `shade_attr()` → composite `attr_t` | Returns `COLOR_PAIR(n) | A_BOLD` etc. in one value |
| Runtime theme swap | `theme_apply(idx)` re-calls `init_pair()` for all 6 pairs — hot-swaps entire gradient on 't' key |
| Two-pass rendering | Pass 1: persistent grid cells (fade trail); Pass 2: `col_paint_interpolated()` floating head positions |
| Float head position | `draw_head_y = (float)c->head_y + (float)c->speed * alpha` — forward extrapolation |
| Round-half-up in cell mapping | `row = (int)floorf(draw_head_y - dist + 0.5f)` |

---

## particle_systems/fire.c

**Purpose:** Doom-style fire cellular automaton.

| Technique | Detail |
|---|---|
| 9-level luminance ramp | `" .:+x*X#@"` — sparse-to-dense ASCII |
| `FireTheme` struct | `fg256[9]`, `fg8[9]`, `attr8[9]` — complete theme encapsulation |
| `ramp_attr(i, theme)` | Returns `COLOR_PAIR(n) | A_BOLD` for top 2 levels in 256-color; `attr8[i]` for 8-color |
| Runtime theme cycling | `theme_apply()` re-registers 9 pairs; `cycle_tick` auto-cycles every 300 simulation ticks |
| Floyd-Steinberg dithering | Error diffused on `heat_work[]` before ramp quantisation |
| Per-cell `attron`/`attroff` | `attron(ramp_attr)` → `mvaddch` → `attroff` — every cell bracketed |

---

## particle_systems/aafire_port.c

**Purpose:** aalib fire variant — minimises terminal write volume.

| Technique | Detail |
|---|---|
| **Diff-based clearing** | **No `erase()`** — only writes `' '` where `prev[i] > 0` and current cell is blank |
| `memcpy(prev, bmap, ...)` | Snapshot after draw — next frame diffs against it |
| Same 9-ramp + 6 themes | Shares the theme struct pattern with `fire.c` |
| 5-neighbour CA | Different diffusion kernel from fire.c — `(c[i-1]+c[i+1]+c[i-W]+c[i+W]+c[i])/5` |
| Per-row decay LUT | Lookup table maps row index to decay probability — faster decay near top |

---

## particle_systems/fireworks.c

**Purpose:** Rocket fireworks state machine (IDLE→RISING→EXPLODED).

| Technique | Detail |
|---|---|
| 7 spectral pairs | 196,208,226,46,51,21,201 — one per hue; same 7 used across brust/kaboom/constellation |
| Life-gated `A_BOLD`/`A_DIM` | `life > 0.6 → A_BOLD`; `life < 0.2 → A_DIM`; mid → base attribute |
| `attr_t` built by OR | `attr_t attr = COLOR_PAIR(p->color); if (...) attr |= A_BOLD;` — accumulate flags |
| Rocket char `'|'` always bold | `COLOR_PAIR(r->color)|A_BOLD` — rising rocket is always brightest |
| Independent particle colors | Each particle gets its own random `COLOR_PAIR` from the 7 spectral pairs |

---

## particle_systems/brust.c

**Purpose:** Random explosion bursts with scorch persistence.

| Technique | Detail |
|---|---|
| Flash cross-pattern | `'*'` at centre + `'+'` in 4 cardinal neighbours — `COLOR_PAIR(C_YELLOW)|A_BOLD` |
| Scorch persistence | `wattron(w, COLOR_PAIR(C_ORANGE)|A_DIM)` drawn every frame — dim residue that never clears until reset |
| Life-gated brightness | `life > 0.65 → A_BOLD`; else base pair — simpler than fireworks (no `A_DIM` fade) |
| Direct `ASPECT=2.0f` | Particle x-spread multiplied by 2.0 in cell space — no pixel-space conversion |
| Same 7 spectral pairs | Reuses the canonical hue set |

---

## particle_systems/kaboom.c

**Purpose:** Deterministic LCG explosions with pre-rendered blast frames.

| Technique | Detail |
|---|---|
| `Cell {ch, ColorID}` pre-render | `blast_render_frame()` fills `Cell[]` array; `blast_draw()` blits to ncurses — render logic fully decoupled |
| 6 blast theme structs | Each has `flash_chars[]` and `wave_chars[]` strings — char selected by blast variant index |
| Z-depth char+color | 3D blob: `bz > 0.8*persp → '.' COL_BLOB_F`; mid → `'o' COL_BLOB_M`; near → `'@' COL_BLOB_N` |
| Role-named color IDs | `COL_BLOB_F/M/N`, `COL_FLASH`, `COL_RING`, `COL_HUD` — semantic colour IDs, not pair numbers |
| HUD always pair 7/yellow | `COL_HUD` is hardcoded yellow regardless of active blast theme |
| LCG seed determinism | Same seed → identical explosion shape every time |

---

## particle_systems/constellation.c

**Purpose:** Star constellation with stippled Bresenham connecting lines.

| Technique | Detail |
|---|---|
| `prev_px`/`prev_py` per star | Lerp: `draw_px = prev_px + (px - prev_px) * alpha` — non-linear orbit uses prev/cur |
| Stippled Bresenham | `if (step_count % stipple == 0) mvaddch(...)` — skips pixels to create dashed lines |
| `bool cell_used[rows][cols]` VLA | Zero-initialised each frame; gates every `mvaddch` — prevents multi-line overdraw artefacts |
| Distance-ratio attributes | `< 0.50 → A_BOLD, stipple=1`; `< 0.75 → normal, stipple=1`; `< 1.00 → normal, stipple=2` — proximity brightness |
| 7 spectral pairs | Same canonical hue set; star colour assigned at birth |

---

## particle_systems/flocking.c

**Purpose:** Boid flocking with 5 algorithm modes and cosine palette.

| Technique | Detail |
|---|---|
| Cosine palette → xterm-256 | `r = 0.5+0.5*cos(2π*(t/period+phase_r))` → mapped to xterm-256 RGB cube index |
| `init_pair()` in animation loop | Palette re-registered every N frames — live colour animation without changing draw calls |
| `follower_brightness()` | Toroidal distance ratio < 0.35 → `A_BOLD`; else `A_NORMAL` — proximity halo |
| `velocity_dir_char()` | Per-flock char set indexed by 8-way heading octant from `atan2(vy,vx)` |
| 5 algorithm modes | `'a'` key cycles: classic boids, leader-chase, Vicsek, orbit, predator-prey — same ncurses draw path |
| Cosine palette cycling | Period and phase offsets per RGB channel create smooth hue rotation over time |

---

## fluid/sand.c

**Purpose:** Falling sand cellular automaton.

| Technique | Detail |
|---|---|
| `grain_visual(age, nb, &ch, &attr)` | Dual-factor visual: age selects density char, neighbour count selects brightness — one function sets both |
| 6 visual levels | `` ` ``(A_BOLD) → `.`(A_BOLD) → `o`(A_BOLD) → `O` → `0` → `#` — density encodes grain compaction |
| Source indicator | `'|'` in `CP_SOURCE|A_BOLD` at emitter position each frame |
| Wind arrow | Arrow char in `CP_WIND` at wind-indicator position |
| Fisher-Yates column shuffle | Randomises update order to prevent directional bias artefacts |
| `KEY_LEFT`/`KEY_RIGHT` | Moves emitter; `KEY_UP`/`KEY_DOWN` adjusts wind — all arrow keys consumed |

---

## fluid/flowfield.c

**Purpose:** Perlin noise flow field with ring-buffer particle trails.

| Technique | Detail |
|---|---|
| 8 color pairs | 4 themes: RAINBOW (8 hue octants), CYAN fade, GREEN fade, WHITE/grey fade |
| `color_apply_theme(theme)` | Re-registers all 8 pairs — 't' key hot-swaps mid-run, same pattern as matrix_rain |
| `angle_to_pair(float angle)` | RAINBOW: `atan2` range → pairs 1–8 by octant; mono themes: trail age → pair (1=newest, 8=oldest) |
| Unicode arrow glyphs | `→↗↑↖←↙↓↘` (8 directions) from `atan2(vy,vx)` octant — stored in `const wchar_t *dirs[]` |
| `addwstr` / `mvaddwstr` | Used for multi-byte Unicode arrows — not plain `mvaddch` |
| Ring-buffer trail | Each particle stores last N positions; older positions use higher pair index (dimmer) |
| 3-octave fBm | Perlin noise composed from 3 octaves for smooth large-scale field structure |
| Bilinear field sampling | Field vector interpolated between grid corners for smooth particle steering |

---

## raster/torus_raster.c

**Purpose:** UV torus with 4 shader modes and software rasteriser.

| Technique | Detail |
|---|---|
| `Cell {ch, color_pair, bold}` + `cbuf[]` | Intermediate framebuffer — all raster math writes here; ncurses never touched during pipeline |
| `zbuf[]` float depth buffer | Initialised to `FLT_MAX`; z-test per cell before overwrite |
| `fb_blit()` sole ncurses boundary | Iterates `cbuf`; skips `ch==0`; `attron(COLOR_PAIR(c.color_pair)|(c.bold?A_BOLD:0))` → `mvaddch` → `attroff` |
| `luma_to_cell(luma, px, py)` | Bayer 4×4 ordered dither → Bourke 92-char ramp → warm-to-cool pair index |
| 7 luminance color pairs | Pair 1=red(196) → pair 7=magenta(201) — luminance-mapped across warm-to-cool spectrum |
| Bayer 4×4 dithering | `bayer[py%4][px%4]` threshold added to luma before ramp lookup |
| Paul Bourke ramp | 92-char `" .,:;i1tfLCG08@"…` string; index = `(int)(luma * 91)` |
| 4 shader modes | Phong, toon, normals, wireframe — same `luma_to_cell` path; only FSIn changes |
| Back-face cull always-on | `dot(face_normal, view_dir) < 0` → skip triangle before rasterise |

---

## raster/cube_raster.c

**Purpose:** Unit cube rasteriser — same pipeline, adds toggleable cull + zoom.

| Technique | Detail |
|---|---|
| Same `cbuf`/`zbuf`/`fb_blit` pipeline | Identical to torus_raster — shared pipeline pattern |
| Toggleable back-face cull | `'c'` key flips `cull_enabled` flag — allows inside-out rendering |
| Zoom key | `'+'/'-'` adjusts FOV constant — updates projection matrix next frame |
| Flat normals | Per-face normals (not per-vertex) — simpler than sphere/torus UV normals |

---

## raster/sphere_raster.c

**Purpose:** UV sphere rasteriser.

| Technique | Detail |
|---|---|
| Same `cbuf`/`zbuf`/`fb_blit` pipeline | Identical pipeline to cube/torus |
| UV sphere tessellation | `tessellate_sphere()` generates vertices from lat/lon loops |
| Toggleable cull + zoom | Same `'c'`/`'+'/'-'` keys as cube_raster |

---

## raster/displace_raster.c

**Purpose:** UV sphere with real-time vertex displacement.

| Technique | Detail |
|---|---|
| Same `cbuf`/`zbuf`/`fb_blit` pipeline | Identical pipeline |
| Vertex displacement modes | Ripple, wave, pulse, spiky — applied in `tessellate` each frame |
| Central-difference normal recompute | Normals recalculated from displaced neighbours each frame — not precomputed |
| `'d'` key cycles displacement | Hot-swaps displacement function — no change to ncurses draw path |

---

## raymarcher/donut.c

**Purpose:** Parametric torus — no mesh, direct trigonometric projection.

| Technique | Detail |
|---|---|
| No framebuffer | Writes directly per computed screen point — no `cbuf[]`, no `fb_blit()` |
| 8 grey-ramp pairs | xterm-256 indices 235,238,241,244,247,250,253,255 (every 3rd grey level, 232–255 range) |
| 8-color fallback | `A_DIM` for dark pairs, base for mid, `A_BOLD` for bright — 3 tiers from 2 attributes |
| `N·L` → pair 1–8 | Dot product of surface normal with light direction maps to grey ramp index |
| Depth sort | Points sorted by z before draw — painter's algorithm for correct overlap |

---

## raymarcher/wireframe.c

**Purpose:** Wireframe cube — Bresenham projected edges.

| Technique | Detail |
|---|---|
| Bresenham 3D projected edges | Each cube edge projected to screen, then Bresenham drawn |
| Slope char per step | Same `(dx&&dy)?(sx==sy?'\\':'/'):dx?'-':'|'` as spring_pendulum |
| `KEY_UP/DOWN/LEFT/RIGHT` rotation | Arrow keys accumulate rotation angles — `nodelay` non-blocking |
| No color pairs | Monochrome — no `start_color()`, no `init_pair` — simplest raymarcher render |

---

## raymarcher/raymarcher.c

**Purpose:** Sphere-marching SDF raymarcher — sphere + infinite plane.

| Technique | Detail |
|---|---|
| Same 8 grey-ramp pairs | 235,238,241,244,247,250,253,255 — identical to donut |
| Gamma correction | `pow(luma, 1.0/2.2)` before ramp lookup — perceptual linearisation |
| Blinn-Phong shading | Specular + diffuse → luma → grey pair |
| `cbuf[]`/`fb_blit()` | Uses intermediate framebuffer unlike donut — full pipeline separation |
| Shadow ray | Secondary march toward light — if hits geometry, surface is in shadow |

---

## raymarcher/raymarcher_cube.c

**Purpose:** SDF box raymarcher.

| Technique | Detail |
|---|---|
| Same 8 grey-ramp pairs + gamma | Identical to raymarcher.c |
| Finite-difference normals | `(sdf(p+ε) - sdf(p-ε)) / 2ε` for each axis — no analytic normal |
| Shadow ray | Same secondary march pattern as raymarcher.c |
| `cbuf[]`/`fb_blit()` | Intermediate framebuffer |

---

## raymarcher/raymarcher_primitives.c

**Purpose:** Multiple SDF primitives composited with min/max.

| Technique | Detail |
|---|---|
| Same 8 grey-ramp pairs + gamma | Identical render path |
| `min`/`max` SDF composition | Union, intersection, subtraction of sphere/box/torus/capsule/cone SDFs |
| Per-primitive color | Each primitive has its own pair — material ID returned alongside distance |
| `cbuf[]`/`fb_blit()` | Intermediate framebuffer |

---

## misc/bonsai.c

**Purpose:** Growing bonsai tree — recursive branch growth, message panel.

| Technique | Detail |
|---|---|
| `use_default_colors()` + `init_pair(n, color, -1)` | `-1` background → transparent — tree on terminal bg |
| ACS line-drawing chars | `ACS_ULCORNER`, `ACS_URCORNER`, `ACS_LLCORNER`, `ACS_LRCORNER`, `ACS_HLINE`, `ACS_VLINE` for message box border |
| `attr_t attr = COLOR_PAIR(cp) | (bo ? A_BOLD : 0)` | Branch boldness gated by boolean flag — older branches dim |
| Slope chars per branch direction | `'|'`, `'-'`, `'/'`, `'\\'` chosen by dx/dy of branch growth step |
| Two HUD rows | `mvprintw(0, hx, ...)` top bar + `mvprintw(rows-1, 0, ...)` bottom status |
| 5 tree types | Different recursive growth parameters — same ncurses draw path |
| 3 pot styles | Different `ACS_*` char combinations for pot outline |
| Message panel | Scrolling text inside ACS box — `mvprintw` with clipped width |

---

## Quick-Reference Matrix

| File | erase() | diff-clear | cbuf+zbuf | grey-ramp | Bayer | FStein | theme-swap | `use_default` | ACS | Unicode |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| tst_lines_cols | — | — | — | — | — | — | — | — | — | — |
| aspect_ratio | wbkgd | — | — | — | — | — | — | — | — | — |
| bounce_ball | ✓ | — | — | — | — | — | — | — | — | — |
| spring_pendulum | ✓ | — | — | — | — | — | — | — | — | — |
| matrix_rain | ✓ | — | — | — | — | — | ✓ | ✓ | — | — |
| fire | ✓ | — | — | — | — | ✓ | ✓ | — | — | — |
| aafire_port | — | ✓ | — | — | — | ✓ | ✓ | — | — | — |
| fireworks | ✓ | — | — | — | — | — | — | — | — | — |
| brust | ✓ | — | — | — | — | — | — | — | — | — |
| kaboom | ✓ | — | — | — | — | — | ✓ | — | — | — |
| constellation | ✓ | — | — | — | — | — | — | — | — | — |
| flocking | ✓ | — | — | — | — | — | ✓ | — | — | — |
| sand | ✓ | — | — | — | — | — | — | — | — | — |
| flowfield | ✓ | — | — | — | — | — | ✓ | — | — | ✓ |
| torus_raster | ✓ | — | ✓ | — | ✓ | — | — | — | — | — |
| cube_raster | ✓ | — | ✓ | — | ✓ | — | — | — | — | — |
| sphere_raster | ✓ | — | ✓ | — | ✓ | — | — | — | — | — |
| displace_raster | ✓ | — | ✓ | — | ✓ | — | — | — | — | — |
| donut | ✓ | — | — | ✓ | — | — | — | — | — | — |
| wireframe | ✓ | — | — | — | — | — | — | — | — | — |
| raymarcher | ✓ | — | ✓ | ✓ | — | — | — | — | — | — |
| raymarcher_cube | ✓ | — | ✓ | ✓ | — | — | — | — | — | — |
| raymarcher_prims | ✓ | — | ✓ | ✓ | — | — | — | — | — | — |
| bonsai | ✓ | — | — | — | — | — | — | ✓ | ✓ | — |

**Column key:**
- `erase()` — standard per-frame full erase
- `diff-clear` — selective cell erase (no full `erase()`)
- `cbuf+zbuf` — intermediate framebuffer + depth buffer, `fb_blit()` as sole ncurses boundary
- `grey-ramp` — xterm-256 grey pairs (235–255 every 3rd)
- `Bayer` — 4×4 ordered dithering before ramp lookup
- `FStein` — Floyd-Steinberg error diffusion
- `theme-swap` — runtime `init_pair()` re-registration to hot-swap colour theme
- `use_default` — `use_default_colors()` for transparent terminal background
- `ACS` — ACS line-drawing characters
- `Unicode` — `addwstr`/`mvaddwstr` for multi-byte glyphs

---

## Technique Index (by technique → files)

| Technique | Files |
|---|---|
| Diff-based clearing (no erase) | aafire_port |
| Intermediate cbuf/zbuf + fb_blit | torus_raster, cube_raster, sphere_raster, displace_raster, raymarcher, raymarcher_cube, raymarcher_primitives |
| Bayer 4×4 ordered dithering | torus_raster, cube_raster, sphere_raster, displace_raster |
| Floyd-Steinberg dithering | fire, aafire_port |
| Paul Bourke 92-char ramp | torus_raster, cube_raster, sphere_raster, displace_raster |
| xterm-256 grey ramp (235–255) | donut, raymarcher, raymarcher_cube, raymarcher_primitives |
| Runtime theme swap (init_pair in loop) | matrix_rain, fire, aafire_port, kaboom, flowfield, flocking |
| `use_default_colors()` transparent bg | matrix_rain, bonsai |
| Cosine palette → xterm-256 cube | flocking |
| `A_BOLD`/`A_DIM` as brightness tiers | fireworks, brust, constellation, flocking, fire, aafire_port, sand |
| Life-gated A_BOLD/A_DIM | fireworks, brust |
| Proximity-gated A_BOLD | constellation, flocking |
| Slope char from Bresenham step | spring_pendulum, wireframe, bonsai |
| ACS line-drawing chars | bonsai |
| Unicode arrows via addwstr | flowfield |
| Stippled Bresenham + cell_used VLA | constellation |
| Pre-rendered Cell[] framebuffer | kaboom |
| Dual-factor visual function | sand (`grain_visual`) |
| Two-pass rendering | matrix_rain |
| Depth sort (painter's algorithm) | donut |
| `raw()` instead of `cbreak()` | aspect_ratio |
| `newwin` manual window | aspect_ratio |
| Aspect x×2 correction | aspect_ratio (circle), brust (`ASPECT=2.0f`) |
| Gamma correction (pow 1/2.2) | raymarcher, raymarcher_cube, raymarcher_primitives |
| Finite-difference SDF normals | raymarcher_cube, raymarcher_primitives |
| `typeahead(-1)` explicit | bounce_ball (documented in comments), all animation files |
