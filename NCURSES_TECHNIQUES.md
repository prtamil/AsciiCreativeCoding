# NCURSES_TECHNIQUES.md — Per-File ncurses Technique Reference

Each section covers one C file. For every notable technique the entry explains:
- **What it is** — one-sentence definition
- **What we achieve** — the visual or behavioral goal
- **How** — the concrete mechanism in plain language

Cross-file universals (initscr, noecho, cbreak, curs_set, nodelay, keypad, atexit/endwin,
SIGINT/SIGWINCH, erase→wnoutrefresh→doupdate loop) are documented in the preamble and
omitted per-file unless the file does something non-standard with them.

---

## Universal Patterns

These appear in almost every animation file. They are listed here once and not repeated per file.

| Pattern | Code form |
|---|---|
| Init sequence | `initscr → noecho → cbreak → curs_set(0) → nodelay(stdscr,TRUE) → keypad(stdscr,TRUE) → typeahead(-1)` |
| Color guard | `if (has_colors()) { start_color(); … }` |
| 256-color fallback | `COLORS >= 256` branch → xterm-256 pairs; else 8-color pairs |
| Atomic diff write | `typeahead(-1)` — disables mid-flush stdin poll |
| Frame sequence | `erase() → draw → wnoutrefresh(stdscr) → doupdate()` |
| Stable frame cap | `clock_sleep_ns()` **before** terminal I/O |
| Signal flags | `volatile sig_atomic_t running, need_resize` |
| Terminal restore | `atexit(cleanup)` → `endwin()` |
| SIGWINCH resize | `endwin() + refresh() + getmaxyx()` on flag |
| Safe mvaddch | `(chtype)(unsigned char)ch` double cast — prevents sign-extension |

---

## ncurses_basics/tst_lines_cols.c

**Purpose:** Minimal `LINES`/`COLS` demo — the absolute simplest ncurses program.

**Overview:** This file deliberately uses almost nothing. Its only job is to show that `initscr()` populates `LINES` and `COLS` with the terminal dimensions. Every technique here is intentionally primitive — it exists to illustrate the baseline before any optimisations are applied.

---

**`LINES` / `COLS` globals**
`LINES` and `COLS` are global integers that ncurses sets to the terminal's row and column count when `initscr()` is called. What we achieve: reading terminal dimensions without needing `getmaxyx()`. How: just read the globals directly — `printw("rows=%d cols=%d", LINES, COLS)` — there is no function call involved.

**`printw` / `refresh()`**
`printw` is ncurses' `printf` equivalent that writes into `newscr` at the current cursor position. What we achieve: the simplest possible text output — one line of text, then flush. How: call `printw(...)` to write into the buffer, then `refresh()` (= `wnoutrefresh + doupdate` in one call) to push it to the terminal.

**Blocking `getch()`**
Without `nodelay(TRUE)`, `getch()` halts the thread until a key is pressed. What we achieve here: the program pauses after printing, waiting for any key before calling `endwin()` and exiting. How: just call `getch()` — it blocks by default. This is the teaching-only usage; all animation files set `nodelay(TRUE)` to prevent this block.

**No signal handling, no `start_color()`**
This file has no `SIGWINCH` handler, no `sig_atomic_t` flags, no color setup. What we achieve: the smallest possible ncurses program — every line is essential. How: simply omit all the safety and color code. This is the "hello world" baseline that all other files build on.

---

## ncurses_basics/aspect_ratio.c

**Purpose:** Demonstrates correct circle drawing by compensating for non-square terminal cells.

**Overview:** Terminal cells are roughly twice as tall as they are wide. If you draw a circle naively, it looks like an ellipse. This file shows both the problem and the fix — multiplying x-coordinates by 2 to compensate. It also demonstrates the `newwin` API pattern, which all later files abandon in favour of `stdscr`.

---

**`newwin(max_y, max_x, 0, 0)`**
`newwin` creates a new `WINDOW*` with specified dimensions at a given screen offset. What we achieve: a separate drawing surface — in this case a window that covers the full terminal. How: after `initscr()`, call `newwin(LINES, COLS, 0, 0)` to get a full-screen window. All draw calls then use `w`-prefixed forms (`wmove`, `waddch`, etc.) targeting that window. This is the pattern all animation files move away from — a single `stdscr` is simpler and avoids diff-engine issues.

**`wbkgd(win, COLOR_PAIR(1))`**
`wbkgd` sets the background attribute for an entire window — every cleared cell gets this attribute. What we achieve: filling the window background with a color rather than the default black. How: call `wbkgd(win, COLOR_PAIR(n))` once after creating the window. Every subsequent `werase()` will fill with that attribute instead of plain spaces.

**`werase(win)` / `wrefresh(win)`**
`werase` clears the window's internal buffer; `wrefresh` stages + flushes it in one call. What we achieve: a complete clear-and-flush cycle for a named window. How: `werase(win)` resets the buffer (no terminal I/O); `wrefresh(win)` does `wnoutrefresh(win)` + `doupdate()` in one step. Later files always use `erase()` (on `stdscr`) + the split `wnoutrefresh`/`doupdate` form for clarity.

**`raw()` instead of `cbreak()`**
`raw()` disables both line buffering and signal generation — `Ctrl+C` arrives as character 3 instead of `SIGINT`. What we achieve: complete keyboard capture including control characters, without triggering signals. How: call `raw()` instead of `cbreak()` in the init sequence. This file uses it because it has no signal handler — it's intentional. All animation files use `cbreak()` so `Ctrl+C` still generates `SIGINT` and triggers the clean shutdown path.

**Aspect correction via x×2**
Terminal cells are ~2× taller than wide. A circle drawn at equal x/y steps looks like a vertical ellipse. What we achieve: a circle that looks visually round. How: multiply the x-component of the circle parametric by 2 — `x = cx + radius * 2 * cos(angle)`. This doubles the horizontal spread to compensate for the cell aspect ratio.

---

## misc/bounce_ball.c

**Purpose:** Reference implementation — the canonical ncurses animation skeleton that all other files follow.

**Overview:** Every architectural decision in this project — pixel-space physics, fixed timestep, render interpolation, the sleep-before-render frame cap, the single-stdscr model — is documented and demonstrated here. When something is unclear in another file, the answer is here.

---

**7 xterm-256 color pairs**
Seven vivid color pairs are registered at startup covering red, orange, yellow, green, cyan, blue, and magenta using specific xterm-256 indices (196, 208, 226, 46, 51, 21, 201). What we achieve: richly colored balls that are visually distinct from each other. How: `init_pair(n, idx, COLOR_BLACK)` for each, with an 8-color fallback branch when `COLORS < 256`.

**`wattron / mvwaddch / wattroff` bracket**
Every character write is surrounded by an attribute activation/deactivation pair: `wattron(w, COLOR_PAIR(b->color)|A_BOLD)` → `mvwaddch(w, cy, cx, ch)` → `wattroff(w, ...)`. What we achieve: each character drawn in its own color with bold brightness. How: the attribute state is active only for the duration of one `mvwaddch` call — this prevents color from leaking into adjacent cells.

**`(chtype)(unsigned char)ch` double cast**
Every `mvwaddch` call wraps the character in `(chtype)(unsigned char)ch`. What we achieve: prevention of sign-extension bugs where high ASCII values (128–255) corrupt the attribute bits of the `chtype` argument. How: cast to `unsigned char` first (zero-extends the value to 8 bits), then to `chtype` (widens safely). One cast alone is insufficient.

**HUD via `mvprintw`**
The FPS counter and status are formatted with `snprintf` into a fixed buffer, then written to row 0 via `mvprintw(0, hud_x, "%s", buf)` with `attron`/`attroff`. What we achieve: a fixed, always-visible overlay. How: the HUD is written last in `screen_draw()`, so it overwrites any scene character at the same cell position — last write wins.

**Pixel-space physics with `floorf` round-half-up**
Physics lives in pixel space (`CELL_W=8`, `CELL_H=16` sub-pixels per cell). The single conversion point is `px_to_cell_x(px) = (int)floorf(px/CELL_W + 0.5f)`. What we achieve: isotropic motion — a ball moving diagonally covers equal physical distance in X and Y. How: `+ 0.5f` before `floorf` implements "round half up", which is deterministic and avoids the oscillation that `roundf` (banker's rounding) produces at cell boundaries.

**Forward render interpolation (alpha)**
Draw position is extrapolated: `draw_px = b->px + b->vx * alpha * dt_sec`. What we achieve: visually smooth motion between physics ticks — no micro-stutter even at 60fps with a 60Hz sim. How: `alpha = sim_accum / tick_ns` is the fraction of the current unfired tick that has elapsed. Projecting forward by `alpha × dt` places the draw position at wall-clock "now."

**`typeahead(-1)` documented in comments**
This file is the only one that includes an inline comment explaining why `typeahead(-1)` is present. What we achieve: a documented reference for future readers. How: the comment explains that it prevents ncurses from polling stdin mid-doupdate(), which would fragment the write and cause tearing.

---

## ncurses_basics/spring_pendulum.c

**Purpose:** Lagrangian spring-pendulum with Bresenham coil rendering and layered draw order.

**Overview:** This file introduces two important patterns not in bounce_ball: semantic color pair names (instead of bare indices) and a deliberate multi-layer draw order where later writes intentionally overwrite earlier ones to produce a physically correct stacked appearance.

---

**5 named color pairs**
Color pairs are given semantic constants: `CP_BAR`, `CP_WIRE`, `CP_SPRING`, `CP_BALL`, `CP_HUD`. What we achieve: readable draw code — `wattron(w, CP_SPRING)` conveys intent; `wattron(w, COLOR_PAIR(3))` does not. How: define enum or `#define` constants mapping semantic names to pair numbers, then use names everywhere in the draw code.

**Slope char from Bresenham step direction**
During Bresenham line traversal, the step direction (dx-only, dy-only, or diagonal) is used to pick the draw character: `(step_x && step_y) ? (sx==sy ? '\\' : '/') : step_x ? '-' : '|'`. What we achieve: a spring coil that looks physically plausible — diagonal segments use `/` and `\`, horizontal ones use `-`, vertical ones use `|`. How: one ternary expression per step — the Bresenham state already tells you the local slope.

**Layered draw order (last write wins)**
The scene is drawn in 6 explicit layers: bar → wire stubs → coil lines → coil nodes → bob. Each layer overwrites cells from the previous. What we achieve: correct visual depth — the bob appears on top of the spring, not hidden by it. How: draw background elements first, foreground elements last. ncurses has no z-buffer; the last `mvwaddch` to a cell is what appears.

**`prev_r`/`prev_theta` lerp for non-linear physics**
The pendulum's angular position is interpolated between the previous and current tick: `draw_r = prev_r + (r - prev_r) * alpha`. What we achieve: smooth motion for non-linear forces where forward extrapolation would overshoot. How: store `prev_r` and `prev_theta` at the start of each tick before advancing. In `scene_draw`, lerp between prev and cur using `alpha` as the weight.

---

## matrix_rain/matrix_rain.c

**Purpose:** Matrix-style falling character rain with 6-shade gradient, transparent background, and hot-swappable themes.

**Overview:** This file introduces three techniques not seen before: transparent terminal background via `use_default_colors()`, a 6-shade `attr_t`-based brightness gradient, and a two-pass frame render that separates the persistent fade texture from the smooth head motion.

---

**`use_default_colors()` + pair `-1` background**
After `start_color()`, call `use_default_colors()` to unlock `-1` as a valid background color in `init_pair`. What we achieve: rain characters float over whatever the terminal's background is — a custom color, wallpaper, or blur effect — with no solid ncurses background fill. How: `init_pair(n, fg_color, -1)` registers a pair with transparent background. This sends ISO 6429 SGR 49 (reset background to default) instead of a specific background color.

**6-shade `Shade` enum → composite `attr_t`**
A `Shade` enum (`FADE`, `DARK`, `MID`, `BRIGHT`, `HOT`, `HEAD`) maps to `shade_attr()` which returns a combined `attr_t`. What we achieve: six visually distinct brightness levels from three ncurses attributes (`A_DIM`, base, `A_BOLD`) across paired color values — a full gradient from barely visible trail to sharp head. How: `shade_attr(s)` returns `COLOR_PAIR(pair_for_shade(s)) | attr_flag_for_shade(s)` in one expression.

**Runtime theme swap via `init_pair()` in the loop**
`theme_apply(idx)` re-registers all 6 color pairs with new xterm-256 indices at runtime. What we achieve: instantly hot-swapping the entire color palette on a keypress, from classic green to white, amber, or blue themes. How: call `init_pair()` for each pair inside `theme_apply()`. ncurses applies the new colors on the next `doupdate()`. No changes needed in the draw code — pair numbers stay the same.

**Two-pass rendering**
Frame rendering is split into Pass 1 (persistent grid texture) and Pass 2 (interpolated column heads). What we achieve: organic fading trails and smooth sub-cell head motion simultaneously — neither pass alone produces both. How: Pass 1 iterates `grid[row][col]` and draws fading cells; Pass 2 computes `draw_head_y = head_y + speed * alpha` and draws the sharp leading character at that float-to-int mapped position.

**Float head position with round-half-up**
The column head position is a float updated each tick. For rendering: `draw_row = (int)floorf(draw_head_y + 0.5f)`. What we achieve: deterministic cell assignment that doesn't oscillate at cell boundaries. How: `+0.5f` before `floorf` is "round half up" — always consistent, unlike `roundf` (banker's rounding) which flips between two cells.

---

## particle_systems/fire.c

**Purpose:** Doom-style fire cellular automaton with 9-level ASCII ramp, Floyd-Steinberg dithering, and auto-cycling themes.

**Overview:** This file introduces the 9-level ASCII brightness ramp, Floyd-Steinberg error diffusion dithering, and a structured `FireTheme` that encapsulates all color + attribute data for one visual style. The theme system is more complex here than in matrix_rain because it must manage 9 pairs rather than 6.

---

**9-level luminance ramp `" .:+x*X#@"`**
A 9-character string ordered from sparse (space = cold) to dense (`@` = hottest). What we achieve: heat is encoded visually as character density — the hotter a cell, the "heavier" the glyph looks. How: map `heat ∈ [0, MAX_HEAT]` to an index 0–8 and use `ramp[index]` as the character for `mvaddch`.

**`FireTheme` struct with `fg256[9]`, `fg8[9]`, `attr8[9]`**
All color and attribute data for one visual theme is bundled into a struct. What we achieve: fully encapsulated themes — switching between fire/ice/lava/etc. requires only swapping which struct is active, not changing any draw logic. How: the struct arrays are indexed by ramp level. `ramp_attr(i, theme)` extracts `COLOR_PAIR(theme->fg256[i])` with appropriate `A_BOLD` for the top tiers.

**Runtime theme cycling with tick counter**
`theme_apply()` re-registers all 9 color pairs. A `cycle_tick` counter auto-calls it every 300 simulation ticks. What we achieve: automatic theme cycling so the fire spontaneously shifts between color palettes. How: increment `cycle_tick` each sim tick; when it reaches the threshold, advance the theme index and call `theme_apply()`.

**Floyd-Steinberg dithering on heat grid**
Before quantizing heat to the 9-level ramp, error diffusion is applied to a scratch copy of the heat grid. What we achieve: smooth heat gradients without the regular banding pattern of ordered dithering — flame tongues look organic rather than posterized. How: process top-to-bottom, left-to-right; for each cell, snap to nearest ramp level, compute the rounding error, distribute it to the right and three lower neighbors using 7/16, 3/16, 5/16, 1/16 weights.

**Per-cell `attron`/`attroff`**
Every `mvaddch` call for a fire cell is bracketed by `attron(ramp_attr(i, theme))` / `attroff(...)`. What we achieve: each cell can have an independent color and brightness attribute based on its heat level. How: there is no "current" global attribute state between cells — each cell sets its own. The cost is two extra function calls per cell, acceptable at terminal resolution.

---

## particle_systems/aafire_port.c

**Purpose:** aalib fire variant — minimises terminal write volume using diff-based clearing instead of `erase()`.

**Overview:** The single most distinctive technique in this file is that it never calls `erase()`. Instead it manually tracks which cells were non-empty last frame and only writes a space character for those that are now empty. This significantly reduces the number of escape codes sent to the terminal per frame compared to the full-screen clear that `erase()` implies.

---

**Diff-based clearing (no `erase()`)**
Instead of `erase()` at the start of each frame, only cells that were non-zero last frame and are now zero get a space written. What we achieve: minimum terminal write volume — only cells that actually changed are transmitted. How: maintain a `prev[rows*cols]` snapshot. Each frame, for any cell where `prev[i] > 0` and the new value is 0, call `mvaddch(y, x, ' ')`. After drawing, `memcpy(prev, bmap, ...)` snapshots the new state for next frame.

**`memcpy(prev, bmap, ...)` snapshot pattern**
After each frame's draw, copy the current heat buffer into `prev`. What we achieve: an exact record of what was drawn last frame, enabling the diff on the next frame. How: one `memcpy` after all `mvaddch` calls for the frame. The snapshot must happen after drawing, not before, otherwise the diff reads stale data.

**5-neighbour CA diffusion kernel**
The fire CA uses 5 neighbours (`y+1, x-1`; `y+1, x`; `y+1, x+1`; `y+2, x-1`; `y+2, x+1`) averaged and decayed. What we achieve: rounder, slower-rising flame blobs compared to fire.c's 4-neighbour version. How: sum the five values, divide by 5, subtract `minus[y]` (the per-row decay LUT).

**Per-row decay LUT**
A precomputed array `minus[rows]` maps row index to decay strength. What we achieve: faster decay near the top of the screen (flames die out at their tips) and slower decay near the bottom (fuel zone stays hot). How: compute the LUT at init time scaled to the terminal height — `minus[y] = max_decay * y / rows`. This way the flame height adapts automatically to any terminal size.

---

## particle_systems/fireworks.c

**Purpose:** Rocket fireworks with a two-level state machine (IDLE→RISING→EXPLODED) and life-gated brightness.

**Overview:** This file introduces the life-gated `A_BOLD`/`A_DIM` pattern — where a particle's brightness attribute is driven by how much of its lifetime remains. Young/hot particles are bold; fading particles are dim. It also shows that particle colors can be assigned independently from their parent rocket color.

---

**7 spectral color pairs**
Seven pairs covering the full visible spectrum: red(196), orange(208), yellow(226), green(46), cyan(51), blue(21), magenta(201). What we achieve: explosions in any color, with a full rainbow available for each burst. How: register all 7 pairs at init. The same 7 pairs are reused across brust.c, kaboom.c, and constellation.c — a shared canonical hue set.

**Life-gated `A_BOLD` / `A_DIM`**
Particle brightness is driven by remaining life: `life > 0.6 → A_BOLD`; `life < 0.2 → A_DIM`; otherwise base pair. What we achieve: particles start bright (young, hot), dim gradually through mid-life, then fade out at the end. How: compute `attr_t attr = COLOR_PAIR(p->color); if (p->life > 0.6f) attr |= A_BOLD; else if (p->life < 0.2f) attr |= A_DIM;` before `mvaddch`.

**`attr_t` accumulation by OR**
The attribute is built incrementally: `attr_t attr = COLOR_PAIR(n); if (condition) attr |= A_BOLD;`. What we achieve: readable conditional attribute logic — the value is assembled in one variable before being applied. How: `COLOR_PAIR(n)` occupies the high bits of `attr_t`; `A_BOLD` and `A_DIM` occupy separate flag bits. Bitwise OR combines them safely.

**Rocket char `'|'` always bold**
The rising rocket is always drawn as `'|'` with `A_BOLD` regardless of other state. What we achieve: the rocket is always the brightest element on screen while rising, drawing the eye to it. How: hardcode `COLOR_PAIR(r->color) | A_BOLD` for the rocket — no life gating needed since the rocket is always "alive" while in RISING state.

**Independent particle colors**
Each particle is assigned a random color from the 7 pairs at spawn, independent of the rocket's color. What we achieve: explosions that burst into multicolored sparks rather than a uniform monochrome cloud. How: `p->color = 1 + rand() % 7` at particle init — no inheritance from parent rocket.

---

## particle_systems/brust.c

**Purpose:** Random explosion bursts with scorch mark persistence and flash cross-pattern.

**Overview:** This file introduces scorch persistence — a simulation-level array that accumulates footprints from past explosions and redraws them as dimmed residue every frame. It also introduces the flash cross-pattern (center `*` plus four cardinal `+` characters) as an immediate visual hit indicator.

---

**Flash cross-pattern**
At the moment of explosion, a `'*'` is drawn at the center cell and `'+'` at the four cardinal neighbors, all with `COLOR_PAIR(C_YELLOW)|A_BOLD`. What we achieve: a sharp, bright impact indicator that communicates "explosion here" in one frame. How: draw the center first, then `mvwaddch(w, cy-1, cx, '+')`, `mvwaddch(w, cy+1, cx, '+')`, etc. — five cells total.

**Scorch persistence with `A_DIM`**
A `scorch[]` array stores the cell positions and characters from past explosion footprints. Every frame, these are drawn first with `COLOR_PAIR(C_ORANGE)|A_DIM`. What we achieve: a screen that accumulates the visual history of all past explosions — faded marks remain visible through subsequent bursts. How: on each explosion, push affected cells to `scorch[]`. Each frame, iterate `scorch[]` and redraw everything in it with the dim attribute. Since `erase()` clears `newscr` each frame, the scorch must be actively redrawn — it is simulation-level persistence, not terminal-level.

**Life-gated brightness (single threshold)**
`life > 0.65 → A_BOLD`; else base pair — no `A_DIM` fade at end of life. What we achieve: simpler visual lifecycle — particles are bright when fresh, then abruptly drop to normal as they age. How: one condition instead of two: `if (p->life > 0.65f) attr |= A_BOLD;`. The scorch system handles the final "death" appearance, so `A_DIM` on dying particles would be redundant.

**Direct `ASPECT=2.0f` in cell space**
Particle x-velocity is multiplied by `ASPECT` at spawn to compensate for non-square cells, directly in cell coordinates. What we achieve: explosion bursts that look circular rather than vertically elongated. How: multiply `vx *= ASPECT` at particle creation. This is simpler than full pixel-space physics — justified because the burst spread is random anyway and doesn't need exact physics accuracy.

---

## particle_systems/kaboom.c

**Purpose:** Deterministic LCG explosions with pre-rendered Cell arrays and 3D blob z-depth coloring.

**Overview:** The central architectural idea here is separating blast rendering from blast drawing: `blast_render_frame()` fills a `Cell[]` array using only simulation logic (no ncurses), and `blast_draw()` blits it to the terminal. This is the same `cbuf`/`fb_blit` pattern from the raster files, applied to a particle system.

---

**`Cell {ch, ColorID}` pre-render buffer**
`blast_render_frame()` fills a flat `Cell[]` array (one entry per screen cell) with character and color data. `blast_draw()` iterates it and calls `attron`/`mvaddch`/`attroff`. What we achieve: complete separation between blast geometry logic and ncurses I/O — the render function is pure C with no ncurses calls. How: define `Cell { char ch; int color_id; }`. Fill during geometry pass; blit in a separate loop.

**6 blast theme structs with `flash_chars[]` / `wave_chars[]`**
Each blast theme defines two character arrays: `flash_chars` (for the initial flash) and `wave_chars` (for ring waves). The character at each element is selected by `chars[variant % len]`. What we achieve: visually distinct blast shapes — fire theme uses `*`, `+`, `o`; ice uses `.`, `*`, `'`; poison uses `%`, `&`, `#`. Same geometry, different glyphs. How: `ch = sh->wave_chars[v % strlen(sh->wave_chars)]` during render.

**3D blob z-depth char + color selection**
Blob elements are computed in 3D space and projected. Z-depth selects both character and color pair: far → `'.' COL_BLOB_F`; mid → `'o' COL_BLOB_M`; near → `'@' COL_BLOB_N`. What we achieve: the blob appears to have depth — distant parts look faded and small-dotted, nearby parts look intense and heavy. How: compute `bz = blob_z_at(angle, radius)`, compare against `persp * 0.8` and `persp * 0.2` thresholds.

**Role-named color IDs**
Colors are named by their role: `COL_BLOB_F` (far blob), `COL_BLOB_M` (mid blob), `COL_BLOB_N` (near blob), `COL_FLASH`, `COL_RING`, `COL_HUD`. What we achieve: the draw code reads like a description of what is being drawn, not an index number. How: define an enum of color IDs; map each to a pair number in `color_init()`.

**HUD always pair 7/yellow regardless of theme**
`COL_HUD` is always registered as yellow regardless of which blast theme is active. What we achieve: the HUD (score, key hints) remains readable no matter what the explosion color palette is. How: `init_pair(COL_HUD, 226, COLOR_BLACK)` is called unconditionally outside the theme-registration block.

**LCG seed determinism**
The blast shape is generated from a seed via a Linear Congruential Generator. What we achieve: the same seed always produces the same explosion — reproducible blasts and an easy replay mechanism. How: `lcg_seed = initial_seed; lcg_next = lcg_seed * 1664525 + 1013904223;` in the geometry loop. Different seeds produce different shapes.

---

## particle_systems/constellation.c

**Purpose:** Star constellation with stippled Bresenham lines, cell deduplication, and distance-based brightness.

**Overview:** This file introduces two techniques unique in this project: stippled Bresenham (drawing only every Nth cell along a line to simulate distance fade) and a `bool cell_used[rows][cols]` VLA that prevents multiple lines from overwriting each other at intersection points.

---

**`prev_px`/`prev_py` per star with lerp**
Each star stores its previous and current position. `draw_px = prev_px + (px - prev_px) * alpha`. What we achieve: smooth orbital motion even though star velocities follow non-linear circular paths (for which forward extrapolation would overshoot). How: save `prev_px = px; prev_py = py;` at the start of each tick, then advance to the new position. In draw, lerp between them with `alpha`.

**Stippled Bresenham — every Nth cell**
During Bresenham line traversal, `mvaddch` is called only when `step_count % stipple == 0`. What we achieve: lines that appear sparser and therefore visually fainter for more distant star pairs — faking opacity without any alpha blending. How: increment `step_count` on every Bresenham advance. `stipple=1` gives a solid line; `stipple=2` skips alternate cells producing a dotted line.

**`bool cell_used[rows][cols]` VLA deduplication**
A per-frame stack-allocated boolean grid tracks which cells have been written this frame. Before any `mvaddch` in the Bresenham loop: `if (!cell_used[y][x]) { draw; cell_used[y][x] = true; }`. What we achieve: clean intersections — when many connection lines cross, each cell shows the first line's character cleanly rather than a collision of overwritten characters. How: VLA on the stack with `bool cell_used[rows][cols]; memset(cell_used, 0, sizeof cell_used);` each frame. `rows` and `cols` are runtime values, hence VLA.

**Distance-ratio attribute + stipple selection**
The Euclidean distance between two stars, divided by a maximum connection distance, gives a ratio 0–1. `ratio < 0.50 → A_BOLD, stipple=1`; `ratio < 0.75 → normal, stipple=1`; `ratio < 1.00 → normal, stipple=2`. What we achieve: three visually distinct connection strengths — close pairs are bright solid lines, distant pairs are faint dotted ones. How: compute `ratio = distance / MAX_CONNECT_DIST` before calling the line draw function, passing `attr` and `stipple` as parameters.

---

## particle_systems/flocking.c

**Purpose:** Boid flocking with 5 switchable algorithm modes, cosine palette color cycling, and proximity brightness.

**Overview:** This file has the most advanced color technique in the project: it calls `init_pair()` inside the animation loop every N frames, re-registering pairs with colors computed from a cosine palette formula. The result is continuously animated colors with no change to the draw path. It also introduces proximity-based `A_BOLD` using toroidal distance.

---

**Cosine palette → xterm-256 cube index**
RGB values are generated using `c = 0.5 + 0.5 * cos(2π * (t/period + phase))` with different phase offsets per channel. Each float is mapped to the xterm-256 6×6×6 color cube: `cube_idx = 16 + 36*ri + 6*gi + bi`. What we achieve: smoothly cycling, perceptually balanced hues that sweep through the full color wheel over time. How: compute `ri = (int)(r * 5 + 0.5f)` etc., then `init_pair(pair_num, cube_idx, COLOR_BLACK)`. The cosine formula guarantees all values stay in [0,1].

**`init_pair()` called inside the animation loop**
Normally `init_pair()` is called once at startup. Here it is called every N frames with new color values. What we achieve: continuously animated flock colors — the boids change hue over time without any change to which color pair numbers are used in draw calls. How: compute the new cube index from the cosine formula, then call `init_pair(pair_num, new_idx, COLOR_BLACK)`. ncurses applies the change on the next `doupdate()`.

**`follower_brightness()` — toroidal proximity halo**
Followers within 35% of the perception radius from their leader get `A_BOLD`; others get `A_NORMAL`. The distance uses toroidal shortest-path. What we achieve: a visual "glow" around each leader — followers cluster around it and appear brighter. How: `float ratio = toroidal_dist(follower, leader) / PERCEPTION_RADIUS; return (ratio < 0.35f) ? A_BOLD : A_NORMAL;`. Toroidal distance is essential — followers near opposite edges are actually close.

**`velocity_dir_char()` per-flock character sets**
Each flock has its own character set indexed by heading octant. `atan2(-vy, vx)` quantizes to 0–7; the result indexes the flock's character array. What we achieve: flocks remain visually distinct even when boids from different flocks overlap in the same screen region. How: define 3 character arrays (one per flock), each with 8 octant characters. Negate `vy` to account for ncurses' inverted Y axis.

---

## fluid/sand.c

**Purpose:** Falling sand CA with dual-factor grain visuals (age × neighbor count) and keyboard-controlled emitter.

**Overview:** The key technique here is `grain_visual()` — a single function that takes both age and neighbor count and sets both the character AND the attribute for a grain. This dual-factor visual encodes two orthogonal properties (how old the grain is, how compacted it is) into one character+attribute combination.

---

**`grain_visual(age, nb, &ch, &attr)` — dual-factor output**
This function takes grain age and neighbor count and writes both a character and an `attr_t` through output pointers. What we achieve: grain appearance encodes two things simultaneously — density character shows compaction, brightness shows age. How: age maps to which character in the 6-level sequence; neighbor count controls whether to apply `A_BOLD`. Both outputs are set inside one function, keeping all visual mapping logic in one place.

**6 visual levels from sparse to dense**
The sequence `` ` ``(A_BOLD) → `.`(A_BOLD) → `o`(A_BOLD) → `O` → `0` → `#` encodes grain compaction as character density. What we achieve: freshly fallen grains look light and sparse; compacted grains at the bottom look heavy and solid. How: the character index advances as the grain's neighbor count increases — more neighbors = denser glyph.

**Source indicator `'|'` in `CP_SOURCE|A_BOLD`**
The emitter position is always drawn as `'|'` with `CP_SOURCE|A_BOLD` each frame. What we achieve: the user can always see where sand is falling from, even when no grains are currently dropping. How: after drawing all grains, always `mvaddch(emitter_row, emitter_col, '|')` with the source attribute. No state check needed — just unconditionally draw it.

**Wind arrow glyph in `CP_WIND`**
The current wind direction and strength is shown as an arrow character at a fixed HUD position using `CP_WIND`. What we achieve: immediate visual feedback for wind state without using text. How: pick an arrow character from `"←↙↓↘→↗↑↖"` based on wind direction, draw it with `CP_WIND` at a corner position.

**Fisher-Yates column shuffle each tick**
The column processing order is randomized each tick using Fisher-Yates shuffle. What we achieve: elimination of the left-to-right scan bias — without this, all sand piles to the right because diagonal-left moves are processed after diagonal-right moves. How: build a `cols[]` index array, shuffle it with Fisher-Yates before the main CA loop, then iterate columns in shuffle order.

---

## fluid/flowfield.c

**Purpose:** Perlin noise flow field with 4 visual themes, Unicode arrow glyphs, and ring-buffer particle trails.

**Overview:** The notable ncurses techniques here are Unicode multi-byte glyph output (using `addwstr` for the 8-direction arrows) and a trail coloring system where older trail positions use higher pair indices (dimmer), encoding time visually through color.

---

**8 color pairs with 4 runtime themes**
Four themes — RAINBOW (8 hue octants), CYAN fade, GREEN fade, WHITE/grey fade — each use 8 pairs. `color_apply_theme(theme)` re-registers all 8 pairs. What we achieve: completely different visual personalities that are swappable on a keypress. How: same pattern as matrix_rain's `theme_apply()` — re-register pairs, take effect on next `doupdate()`.

**`angle_to_pair(float angle)` — hue or age**
In RAINBOW theme, the particle's velocity angle maps to one of 8 hue pairs via octant. In mono themes, the trail position index maps to a pair (1 = newest/brightest, 8 = oldest/dimmest). What we achieve: two completely different uses of the same 8 pairs — direction encoding vs time encoding — switched by theme. How: the function branches on the current theme. For RAINBOW, `pair = 1 + (int)((angle + M_PI) / (M_PI/4)) % 8`.

**Unicode arrow glyphs via `mvaddwstr`**
Direction arrows `→↗↑↖←↙↓↘` are multi-byte UTF-8 sequences stored as `const wchar_t *dirs[8]`. What we achieve: visually intuitive direction indicators — each arrow genuinely points the way the flow pushes particles. How: use `move(y, x); addwstr(dirs[octant]);` instead of `mvaddch`. `addwstr` handles the multi-byte sequence correctly. The `mvadd_wch` or `mvaddwstr` variants require the terminal to be in a UTF-8 locale.

**Ring-buffer trail coloring**
Each particle keeps a ring buffer of its last N positions. The draw loop iterates from newest (index 0) to oldest (index N-1), using `pair = 1 + trail_age_index`. What we achieve: trails that visually fade from bright (newest) to dim (oldest) — the particle's recent history is visible as a color gradient. How: `attron(COLOR_PAIR(1 + i)); mvaddch(trail[i].y, trail[i].x, '.');` where `i` is position in the ring.

---

## raster/torus_raster.c

**Purpose:** UV torus software rasteriser with 4 shader modes, Bayer dithering, and warm-to-cool luminance coloring.

**Overview:** This file establishes the intermediate framebuffer pattern: all rasterisation math writes to `cbuf[]` and `zbuf[]`; a single `fb_blit()` function converts that to ncurses at the end of the frame. This decouples 3D pipeline logic from terminal I/O completely.

---

**`Cell {ch, color_pair, bold}` + `cbuf[]` intermediate framebuffer**
All rasterisation (barycentric interpolation, z-test, fragment shading) writes `Cell` structs into a flat array indexed by `y * cols + x`. What we achieve: the entire 3D pipeline operates without touching ncurses at all — it is pure C arithmetic. How: each pipeline stage reads/writes `cbuf[]`. ncurses is only called from `fb_blit()`, which is the single well-defined boundary between pipeline and display.

**`zbuf[]` float depth buffer**
Initialized to `FLT_MAX` each frame; each rasterised pixel writes to `zbuf[y*cols+x]` only if the new z-value is less than the stored one. What we achieve: correct depth ordering — nearer triangles overwrite farther ones. How: `if (frag_z < zbuf[idx]) { zbuf[idx] = frag_z; cbuf[idx] = cell; }` in the rasteriser loop.

**`fb_blit()` — sole ncurses boundary**
Iterates `cbuf[rows*cols]`, skips cells where `ch == 0` (empty, no geometry covered that cell), and calls `attron(COLOR_PAIR(c.color_pair) | (c.bold ? A_BOLD : 0))` → `mvaddch` → `attroff` for each non-empty cell. What we achieve: all ncurses state changes happen in one tight loop at one function — easy to audit, easy to extend. How: `fb_blit()` is called once at the end of `screen_draw()`, after the entire pipeline has finished.

**`luma_to_cell(luma, px, py)` — dither + ramp + color**
Given a float luminance and cell coordinates, this function applies Bayer dithering, maps to the Bourke ramp character, and picks a warm-to-cool color pair index. What we achieve: a single function that converts a float brightness into the full ncurses output specification (char + pair + bold) in one place. How: `dithered = luma + (bayer[py%4][px%4] - 0.5) * 0.15`; `idx = (int)(dithered * 91)`; `ch = k_bourke[idx]`; `cp = 1 + (int)(dithered * 6)`.

**Bayer 4×4 ordered dithering**
The Bayer matrix threshold at `(px%4, py%4)` is added to the luminance before the ramp lookup. What we achieve: gradients render as a halftone spatial pattern rather than flat banding — the surface appears to have smooth shading even with only 92 ASCII density levels. How: the 4×4 matrix cycles every 4 pixels in both directions, adding a position-dependent offset that breaks up the uniform quantization steps.

**7 warm-to-cool luminance color pairs**
Pair 1 = red(196, warm/bright) → pair 4 = green(neutral) → pair 7 = magenta(201, cool/dim). High luminance maps to low pair index (warm), low luminance maps to high pair index (cool). What we achieve: 3D surfaces look physically plausible — lit areas appear warm/yellowish, shadow areas appear cool/blueish, approximating real light source color temperature. How: `cp = clamp(1 + (int)(dithered_luma * 6), 1, 7)`.

**Back-face culling always-on**
Every triangle with a surface normal pointing away from the camera is skipped before rasterisation. `dot(face_normal, view_dir) < 0 → cull`. What we achieve: hollow objects like a torus look solid — you only see the faces pointing toward you. How: compute the 2D signed area of the projected triangle; if negative, skip the entire rasterisation loop for that triangle.

---

## raster/cube_raster.c

**Purpose:** Unit cube rasteriser — same pipeline as torus_raster with toggleable back-face cull and zoom.

**Overview:** The core pipeline is identical to torus_raster. The additions are interactive keys: `'c'` toggles back-face culling (allowing inside-out rendering) and `'+'/'-'` adjust the field of view. These demonstrate how the same `cbuf`/`fb_blit` pipeline handles different geometry and interactivity without any change to the ncurses output layer.

---

**Same `cbuf`/`zbuf`/`fb_blit` pipeline**
Identical intermediate framebuffer approach from torus_raster. What we achieve: consistent decoupling of 3D logic from ncurses output. How: copy the pipeline unchanged — only `tessellate_cube()` and the normal computation differ.

**Toggleable back-face cull via `'c'` key**
A `cull_enabled` boolean is flipped on each `'c'` keypress and checked before the cull test. What we achieve: toggling between seeing only outer faces (cull on) and seeing inner faces too (cull off) — inside-out rendering that reveals the cube's interior. How: `if (cull_enabled && dot(N, V) < 0) continue;` in the rasteriser.

**Zoom via `'+'/'-'` adjusting FOV**
`KEY_PLUS`/`KEY_MINUS` increment/decrement a `fov` constant that scales the perspective projection matrix. What we achieve: interactive zoom on the cube without changing physics or geometry. How: the projection matrix is recomputed each frame from `fov` — changing `fov` takes effect on the next frame automatically.

---

## raster/sphere_raster.c

**Purpose:** UV sphere rasteriser — same 4 shaders and pipeline as cube/torus.

**Overview:** Identical pipeline to cube_raster and torus_raster. The difference is only the tessellation function (`tessellate_sphere()` with lat/lon loops). Demonstrates the pipeline's generality — any mesh can be dropped in without changing the ncurses output layer.

---

**Same `cbuf`/`zbuf`/`fb_blit` pipeline**
Identical to torus_raster and cube_raster. What we achieve: consistent, reusable ncurses output pattern across all raster files. How: the pipeline source is shared — only geometry generation differs.

**UV sphere tessellation**
`tessellate_sphere()` generates vertices on a latitude/longitude grid: `x = sin(lat)*cos(lon)`, `y = cos(lat)`, `z = sin(lat)*sin(lon)`. What we achieve: a smooth spherical mesh from a regular parametric grid. How: nested loops over `lat ∈ [0, π]` and `lon ∈ [0, 2π]` with configurable step counts. UV coordinates `(u, v) = (lon/2π, lat/π)` are set for each vertex.

---

## raster/displace_raster.c

**Purpose:** UV sphere with real-time vertex displacement — the most complex raster file.

**Overview:** This file re-tessellates the sphere every frame with displaced vertex positions. The normals cannot be precomputed — they must be recomputed from the displaced surface using central differences each frame. Changing displacement modes is a single-key operation that swaps a function pointer, with no change to the ncurses output layer.

---

**Same `cbuf`/`zbuf`/`fb_blit` pipeline**
Identical pipeline. What we achieve: real-time 3D displacement mapping rendered to ASCII with no change to the ncurses output layer. How: the displacement is applied entirely within `tessellate_displace()` — `fb_blit()` is called identically.

**Vertex displacement modes via function pointer**
Four displacement functions (RIPPLE, WAVE, PULSE, SPIKY) are stored as `float (*disp_fn)(Vec3 pos, float t, float amp, float freq)`. `'d'` key cycles `mode_idx` which selects the active function pointer. What we achieve: instant hot-swap of displacement style — same mesh topology, completely different surface shape. How: `disp_fn = displace_fns[mode_idx % N_MODES]` is set before tessellation each frame.

**Central-difference normal recomputation**
After displacing each vertex, normals are recomputed from the displaced neighbors: `T' = T*(2*eps) + N*d_t; B' = B*(2*eps) + N*d_b; N' = normalize(cross(T', B'))`. What we achieve: accurate lighting on the displaced surface — without this, all normals point radially outward like an undisplaced sphere and the shading looks wrong. How: compute `d_t = disp(pos + eps*T) - disp(pos - eps*T)` and `d_b` similarly, then compute the new tangent/bitangent and cross-product.

---

## raymarcher/donut.c

**Purpose:** Parametric torus rendered by direct trigonometric projection — no mesh, no SDF, no framebuffer.

**Overview:** This is the classic "donut.c" algorithm. It differs from all other 3D files in that it writes directly to ncurses per computed point with no intermediate buffer. The 8 grey-ramp pairs provide a luminance gradient; depth sort handles correct overlap.

---

**No intermediate framebuffer — direct ncurses writes**
Each computed screen point calls `mvaddch` directly inside the render loop. What we achieve: the simplest possible 3D render path — no `cbuf[]`, no `fb_blit()`, no pipeline abstraction. How: compute `(cx, cy)` from the parametric torus formula, compute luminance from `N·L`, look up the character and pair, and call `mvaddch` immediately. Works because each parametric point maps to a unique screen cell.

**8 grey-ramp pairs (235, 238, 241, 244, 247, 250, 253, 255)**
Eight color pairs registered to evenly-spaced grey levels from the xterm-256 grey ramp. What we achieve: a visible luminance gradient — dark-shadowed areas use dark grey pairs, brightly lit areas use near-white pairs. How: register `init_pair(n, 235 + (n-1)*3, COLOR_BLACK)` for n = 1..8. Map the `N·L` dot product to a pair index `1 + (int)(luma * 7)`.

**8-color fallback: `A_DIM`/base/`A_BOLD` as 3 tiers**
When `COLORS < 256`, the grey ramp is approximated using `A_DIM` (dark third), base (mid third), `A_BOLD` (bright third). What we achieve: a working 3-level brightness gradient on 8-color terminals. How: `if (pair_idx < 3) attr |= A_DIM; else if (pair_idx > 5) attr |= A_BOLD;`.

**Depth sort (painter's algorithm)**
All torus points for the current frame are computed, sorted by z-depth, then drawn back-to-front. What we achieve: correct overlap — points on the near side of the torus overwrite points on the far side. How: store all computed points in a buffer, sort by `z` descending, then iterate and call `mvaddch` in order. No z-buffer needed because only one point per screen cell is drawn.

---

## raymarcher/wireframe.c

**Purpose:** Wireframe cube with real-time 3D rotation via Bresenham projected edges and slope characters.

**Overview:** The simplest 3D file. No color pairs, no alpha, no framebuffer — just Bresenham lines between projected vertices with slope-based character selection. The entire visual effect comes from choosing `/`, `\`, `-`, `|` based on the direction of each line segment step.

---

**Bresenham projected edges**
Each of the 12 cube edges is drawn by projecting both endpoint vertices to screen space and connecting them with Bresenham's line algorithm. What we achieve: a wireframe cube that rotates in 3D space. How: for each edge `(v0, v1)`, project both to `(cx0, cy0)` and `(cx1, cy1)`, then run Bresenham between the two screen points.

**Slope char per Bresenham step**
At each Bresenham step, the draw character is selected from the step direction: `dx&&dy && sx==sy → '\\'`; `dx&&dy && sx!=sy → '/'`; `dx only → '-'`; `dy only → '|'`. What we achieve: each edge looks like a wire drawn at its actual angle rather than a string of identical characters. How: the same one-expression slope selector from spring_pendulum.c.

**Arrow key rotation — `nodelay` non-blocking**
`KEY_UP`, `KEY_DOWN`, `KEY_LEFT`, `KEY_RIGHT` accumulate rotation angles each frame. What we achieve: continuous smooth rotation while a key is held, with instant response when released. How: `nodelay(TRUE)` makes `getch()` return `ERR` when no key is held — the rotation only advances when a key is actively pressed.

**Monochrome — no `start_color()`**
This file calls `initscr()` but not `start_color()`. What we achieve: the absolute minimal rendering setup — no color init, no pair registration, no attribute logic. How: omit `start_color()` entirely. All `mvaddch` calls use the default attribute. Demonstrates that ncurses works fine without any color setup.

---

## raymarcher/raymarcher.c

**Purpose:** Sphere-marching SDF raymarcher — sphere + infinite plane, Blinn-Phong, soft shadow.

**Overview:** This is the SDF raymarcher baseline. It shares the same 8 grey-ramp pairs as donut.c but adds gamma correction, Blinn-Phong specular, a shadow ray, and the `cbuf[]`/`fb_blit()` intermediate framebuffer pattern from the raster files.

---

**Same 8 grey-ramp pairs + gamma correction**
Same pairs as donut (235, 238, ..., 255). Gamma correction `pow(luma, 1.0/2.2)` is applied before the pair index lookup. What we achieve: perceptually uniform brightness steps — without gamma correction the dark grey pairs are disproportionately dark and mid-tones are compressed. How: `luma_gamma = powf(raw_luma, 1.0f/2.2f); pair_idx = 1 + (int)(luma_gamma * 7);`.

**Blinn-Phong shading → luma → grey pair**
Diffuse (`N·L`) + specular (`(N·H)^shininess`) → combined luma → pair index. What we achieve: 3D lit surfaces with visible highlights and smooth diffuse falloff — the sphere looks round. How: compute `H = normalize(L + V)`; `specular = pow(max(dot(N, H), 0), 32)`; `luma = kd * diffuse + ks * specular`.

**`cbuf[]`/`fb_blit()` unlike donut**
Unlike donut.c which writes directly to ncurses, this file uses the intermediate `cbuf[]` + `fb_blit()` pattern from the raster files. What we achieve: the SDF march loop is pure C arithmetic with no ncurses calls; the output boundary is isolated in one function. How: fill `cbuf[cy*cols + cx] = {ch, pair, bold}` per ray, then call `fb_blit()` once after all rays complete.

**Shadow ray — secondary march toward light**
After computing a hit point, launch a secondary ray from `p + N*eps` toward the light source. If it hits before reaching the light, the surface is in shadow. What we achieve: hard shadows — areas blocked from the light source appear significantly darker. How: march the shadow ray with the same SDF. If `t < dist_to_light`, multiply diffuse by 0 (or a shadow softness factor).

---

## raymarcher/raymarcher_cube.c

**Purpose:** SDF box raymarcher — same pipeline as raymarcher.c, adds finite-difference normals.

**Overview:** The only meaningful addition over raymarcher.c is the normal computation method. Instead of an analytic formula (the sphere's normal is just the normalized point-minus-center), the cube SDF's normal is computed by finite differences — sampling the SDF at 6 perturbed points.

---

**Finite-difference SDF normals**
`N = normalize((sdf(p+eps,0,0)-sdf(p-eps,0,0), sdf(p+0,eps,0)-sdf(p-0,eps,0), sdf(p+0,0,eps)-sdf(p-0,0,eps)))`. What we achieve: correct surface normals for any SDF shape without needing to derive the analytic gradient — a general solution. How: 6 extra SDF evaluations per hit point. The `eps` value (typically 0.001) must be small enough to approximate the true gradient but large enough to avoid floating-point cancellation.

**Same 8 grey-ramp pairs + gamma**
Identical to raymarcher.c. What we achieve: the same luminance-to-grey pair mapping works for any SDF shape. How: the only difference is the SDF function and normal method — the color and output pipeline is unchanged.

---

## raymarcher/raymarcher_primitives.c

**Purpose:** Multiple SDF primitives (sphere, box, torus, capsule, cone) composited with min/max/smin, each with its own color pair.

**Overview:** This file adds SDF composition (union, intersection, subtraction) and per-primitive material colors. The march loop returns both a distance and a material ID. The material ID maps to a specific grey-ramp pair, giving each primitive a distinct brightness level.

---

**`min`/`max` SDF composition**
Union = `min(sdf_a, sdf_b)`; intersection = `max(sdf_a, sdf_b)`; subtraction = `max(-sdf_a, sdf_b)`; smooth union = `smin(sdf_a, sdf_b, k)`. What we achieve: complex compound shapes built from simple primitives — a sphere with a box carved out, two merged blobs, etc. How: the march uses `map(p)` which applies all composition operations and returns the minimum distance across all combined SDFs.

**Per-primitive material color**
`map(p)` returns both the minimum SDF distance and a material ID identifying which primitive was closest. The material ID maps to a grey-ramp pair. What we achieve: each primitive renders in a distinct shade — sphere in light grey, box in mid grey, torus in dark grey — making the composition visually legible. How: `if (d_sphere < d_box) { min_d = d_sphere; mat_id = MAT_SPHERE; } else { ... }`.

**`cbuf[]`/`fb_blit()` with per-material pair**
Same intermediate framebuffer as raymarcher.c. What we achieve: material ID is stored in `cbuf[idx].color_pair` so `fb_blit()` automatically uses the right pair per cell. How: `cbuf[idx] = { ch, mat_id_to_pair[mat_id], bold }`.

---

## misc/bonsai.c

**Purpose:** Growing bonsai tree with recursive branch growth, transparent background, ACS box drawing, and dual HUD rows.

**Overview:** The two most distinct ncurses techniques here are: `use_default_colors()` with `-1` backgrounds (tree branches over transparent terminal background) and `ACS_*` line-drawing characters (portable box border for the message panel without hardcoding Unicode).

---

**`use_default_colors()` + `init_pair(n, color, -1)`**
After `start_color()`, call `use_default_colors()` then register all tree color pairs with `-1` as background. What we achieve: branches and leaves float over the terminal's native background — the tree looks like it's growing in whatever environment the user has set. How: `-1` background in `init_pair` maps to ISO 6429 SGR 49 (default background), which the terminal renders transparently.

**ACS line-drawing characters for message box**
`ACS_ULCORNER`, `ACS_URCORNER`, `ACS_LLCORNER`, `ACS_LRCORNER`, `ACS_HLINE`, `ACS_VLINE` build the message panel border. What we achieve: a clean, portable box outline that renders as Unicode box-drawing chars on modern terminals and uses the VT100 alternate character set on legacy ones — no hardcoded Unicode needed. How: draw corners first, then fill the top/bottom edges with `ACS_HLINE` in a loop, draw the sides with `ACS_VLINE`.

**`attr_t attr = COLOR_PAIR(cp) | (bo ? A_BOLD : 0)`**
Branch boldness is controlled by a boolean flag passed through the growth system. What we achieve: older, thicker branches draw bolder; young, thin branches draw at base brightness — visual age differentiation. How: build the `attr_t` by OR-ing bold conditionally: `attr_t a = COLOR_PAIR(cp) | (bold ? A_BOLD : 0)`.

**Slope chars per branch direction**
Branch characters are selected by the ratio `|dx| / |dy|`: `|dx| << |dy| → '|'`; `|dy| << |dx| → '-'`; `dx*dy > 0 → '\\'`; else `'/'`. What we achieve: branches look like natural wood — predominantly vertical trunks use `|`, lateral branches use `-`, diagonal growth uses `/` and `\`. How: compare absolute values of growth direction components with a factor-of-2 threshold to distinguish diagonal from axial.

**Dual HUD rows**
`mvprintw(0, hx, ...)` writes the top-bar status; `mvprintw(rows-1, 0, ...)` writes the bottom-bar status. What we achieve: more HUD information without crowding either edge — type/seed info at top, growth progress at bottom. How: use two separate `mvprintw` calls with different row coordinates, both drawn after `scene_draw()`.

**Message panel scrolling text**
The message box contains scrolling text drawn with `mvprintw` clipped to `box_w - 2` characters. What we achieve: a log or message display inside an ACS-bordered box, updated each growth cycle. How: use `snprintf(buf, box_w - 1, "%s", message)` to pre-clip, then `mvprintw(by+1, bx+1, "%s", buf)`.

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
| Dual-factor visual function | sand (grain_visual) |
| Two-pass rendering | matrix_rain |
| Depth sort (painter's algorithm) | donut |
| `raw()` instead of `cbreak()` | aspect_ratio |
| `newwin` manual window | aspect_ratio |
| Aspect x×2 correction | aspect_ratio (circle), brust (ASPECT=2.0f) |
| Gamma correction (pow 1/2.2) | raymarcher, raymarcher_cube, raymarcher_primitives |
| Finite-difference SDF normals | raymarcher_cube, raymarcher_primitives |
| `typeahead(-1)` explicit | bounce_ball (documented in comments), all animation files |
