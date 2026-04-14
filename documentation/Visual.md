# Visual.md — ncurses Quirks, Techniques & Ideas

All ncurses-specific knowledge distilled from every C file in this project.
Organised as a field guide: what the call does, why it matters, where it appears.

---

## Index

### V1 — Session Lifecycle
- [V1.1 Standard Initialization Sequence](#v11-standard-initialization-sequence)
- [V1.2 `endwin()` and `atexit` Cleanup](#v12-endwin-and-atexit-cleanup)
- [V1.3 `use_default_colors()` — Transparent Background](#v13-use_default_colors--transparent-background)
- [V1.4 `cbreak()` vs `raw()`](#v14-cbreak-vs-raw)
- [V1.5 Monochrome Mode — Omitting `start_color()`](#v15-monochrome-mode--omitting-start_color)

### V2 — Double Buffer Mechanics
- [V2.1 `curscr` / `newscr` — The Built-in Buffer Pair](#v21-curscr--newscr--the-built-in-buffer-pair)
- [V2.2 `erase()` vs `clear()` — Never Use `clear()` in a Loop](#v22-erase-vs-clear--never-use-clear-in-a-loop)
- [V2.3 `wnoutrefresh` + `doupdate` — The Two-Phase Flush](#v23-wnoutrefresh--doupdate--the-two-phase-flush)
- [V2.4 Why Manual Front/Back Windows Break the Diff Engine](#v24-why-manual-frontback-windows-break-the-diff-engine)
- [V2.5 Framebuffer-to-ncurses: `cbuf` → `fb_blit()`](#v25-framebuffer-to-ncurses-cbuf--fb_blit)
- [V2.6 Diff-based Selective Clearing — No `erase()`](#v26-diff-based-selective-clearing--no-erase)

### V3 — Color System
- [V3.1 Color Pairs — `init_pair` / `COLOR_PAIR`](#v31-color-pairs--init_pair--color_pair)
- [V3.2 256-Color vs 8-Color Fallback Pattern](#v32-256-color-vs-8-color-fallback-pattern)
- [V3.3 xterm-256 Palette Index Reference](#v33-xterm-256-palette-index-reference)
- [V3.4 `use_default_colors()` + Background `-1`](#v34-use_default_colors--background--1)
- [V3.5 Grey Ramp for Luminance Gradients](#v35-grey-ramp-for-luminance-gradients)
- [V3.6 `attr_t` — Combining Pair and Attribute Flags](#v36-attr_t--combining-pair-and-attribute-flags)
- [V3.7 A_BOLD / A_DIM as Brightness Tiers](#v37-a_bold--a_dim-as-brightness-tiers)
- [V3.8 Dynamic Color Update — Cosine Palette (flocking.c)](#v38-dynamic-color-update--cosine-palette-flockingc)
- [V3.9 Shade Enum → Composite `attr_t` Gradient](#v39-shade-enum--composite-attr_t-gradient)
- [V3.10 Encapsulated Theme Struct](#v310-encapsulated-theme-struct)
- [V3.11 Life-gated `A_BOLD` / `A_DIM`](#v311-life-gated-a_bold--a_dim)
- [V3.12 Role-named Color IDs](#v312-role-named-color-ids)

### V4 — Character Output
- [V4.1 `mvwaddch` — The Core Write Call](#v41-mvwaddch--the-core-write-call)
- [V4.2 `(chtype)(unsigned char)` Double Cast](#v42-chtypeunsigned-char-double-cast)
- [V4.3 `wattron` / `wattroff` vs `attron` / `attroff`](#v43-wattron--wattroff-vs-attron--attroff)
- [V4.4 `mvprintw` for HUD Text](#v44-mvprintw-for-hud-text)
- [V4.5 ACS Line-Drawing Characters](#v45-acs-line-drawing-characters)
- [V4.6 Paul Bourke ASCII Density Ramp](#v46-paul-bourke-ascii-density-ramp)
- [V4.7 Directional Characters — Velocity to Glyph](#v47-directional-characters--velocity-to-glyph)
- [V4.8 Branch/Slope Characters — Angle to `/`, `\`, `|`, `-`](#v48-branchslope-characters--angle-to----)
- [V4.9 Unicode Multi-byte Glyphs via `addwstr`](#v49-unicode-multi-byte-glyphs-via-addwstr)

### V5 — Visual Effects Techniques
- [V5.1 Draw Order / Z-Ordering — Last Write Wins](#v51-draw-order--z-ordering--last-write-wins)
- [V5.2 HUD Always On Top](#v52-hud-always-on-top)
- [V5.3 Two-Pass Rendering (matrix_rain.c)](#v53-two-pass-rendering-matrix_rainc)
- [V5.4 Stippled Bresenham Lines — Distance Fade (constellation.c)](#v54-stippled-bresenham-lines--distance-fade-constellationc)
- [V5.5 `cell_used[][]` Grid — Preventing Line Overdraw](#v55-cell_used-grid--preventing-line-overdraw)
- [V5.6 Proximity Brightness — `A_BOLD` by Distance (flocking.c)](#v56-proximity-brightness--a_bold-by-distance-flockingc)
- [V5.7 Bayer 4×4 Ordered Dithering → ASCII Density](#v57-bayer-44-ordered-dithering--ascii-density)
- [V5.8 Floyd-Steinberg Error Diffusion → ASCII (fire.c)](#v58-floyd-steinberg-error-diffusion--ascii-firec)
- [V5.9 Luminance-to-Color Mapping](#v59-luminance-to-color-mapping)
- [V5.10 Scorch Mark Accumulation (brust.c)](#v510-scorch-mark-accumulation-brustc)
- [V5.11 Flash Cross-pattern — Instant Impact Indicator](#v511-flash-cross-pattern--instant-impact-indicator)
- [V5.12 Z-depth → Character + Color Selection](#v512-z-depth--character--color-selection)
- [V5.13 Dual-factor Visual Function](#v513-dual-factor-visual-function)
- [V5.14 Ring-buffer Trail Coloring](#v514-ring-buffer-trail-coloring)
- [V5.15 `luma_to_cell` — Dither + Ramp + Color in One Call](#v515-luma_to_cell--dither--ramp--color-in-one-call)
- [V5.16 Depth Sort — Painter's Algorithm](#v516-depth-sort--painters-algorithm)

### V6 — Input Handling
- [V6.1 `nodelay` — Non-blocking Input](#v61-nodelay--non-blocking-input)
- [V6.2 `keypad(stdscr, TRUE)` — Function and Arrow Keys](#v62-keypadstdscr-true--function-and-arrow-keys)
- [V6.3 `typeahead(-1)` — Prevent Mid-flush Poll](#v63-typeahead-1--prevent-mid-flush-poll)
- [V6.4 `noecho()` — Suppress Key Echo](#v64-noecho--suppress-key-echo)
- [V6.5 `curs_set(0)` — Hide the Cursor](#v65-curs_set0--hide-the-cursor)

### V7 — Signal Handling & Resize
- [V7.1 `SIGWINCH` — Terminal Resize Signal](#v71-sigwinch--terminal-resize-signal)
- [V7.2 `volatile sig_atomic_t` — Signal-safe Flags](#v72-volatile-sig_atomic_t--signal-safe-flags)
- [V7.3 `endwin() → refresh() → getmaxyx()` Resize Sequence](#v73-endwin--refresh--getmaxyx-resize-sequence)
- [V7.4 `atexit(cleanup)` — Guaranteed Terminal Restore](#v74-atexitcleanup--guaranteed-terminal-restore)

### V8 — Performance & Correctness
- [V8.1 Sleep Before Render — Stable Frame Cap](#v81-sleep-before-render--stable-frame-cap)
- [V8.2 `getmaxyx` vs `LINES`/`COLS`](#v82-getmaxyx-vs-linescols)
- [V8.3 `CLOCK_MONOTONIC` — No NTP Jumps](#v83-clock_monotonic--no-ntp-jumps)
- [V8.4 Common ncurses Bugs and How This Project Avoids Them](#v84-common-ncurses-bugs-and-how-this-project-avoids-them)

### V9 — Per-File Technique Reference
- [tst_lines_cols.c](#tst_lines_colsc)
- [aspect_ratio.c](#aspect_ratioc)
- [bounce_ball.c](#bounce_ballc)
- [spring_pendulum.c](#spring_pendulumc)
- [matrix_rain.c](#matrix_rainc)
- [fire.c](#firec)
- [aafire_port.c](#aafire_portc)
- [fireworks.c](#fireworksc)
- [brust.c](#brustc)
- [kaboom.c](#kaboomc)
- [constellation.c](#constellationc)
- [flocking.c](#flockingc)
- [sand.c](#sandc)
- [flowfield.c](#flowfieldc)
- [torus_raster.c](#torus_rasterc)
- [cube_raster.c](#cube_rasterc)
- [sphere_raster.c](#sphere_rasterc)
- [displace_raster.c](#displace_rasterc)
- [donut.c](#donutc)
- [wireframe.c](#wireframec)
- [raymarcher.c](#raymarcherc)
- [raymarcher_cube.c](#raymarcher_cubec)
- [raymarcher_primitives.c](#raymarcher_primitivesc)
- [bonsai.c](#bonsaic)
- [wator.c](#watorc)
- [sandpile.c](#sandpilec)
- [metaballs.c](#metaballsc)
- [lissajous.c](#lissajousc)
- [wave_interference.c](#wave_interferencec)
- [led_number_morph.c](#led_number_morphc)
- [particle_number_morph.c](#particle_number_morphc)
- [julia_explorer.c](#julia_explorerc)
- [barnsley.c](#barnsleyc)
- [diffusion_map.c](#diffusion_mapc)
- [tree_la.c](#tree_lac)
- [lyapunov.c](#lyapunovc)
- [barnes_hut.c](#barnes_hutc)

### Reference Tables
- [Quick-Reference Matrix](#quick-reference-matrix)
- [Technique Index](#technique-index)

---

## Entries

---

### V1 — Session Lifecycle

#### V1.1 Standard Initialization Sequence

**What it is:** A fixed sequence of ncurses setup calls that must happen once at program start before any drawing or input.

**What we achieve:** A fully controlled terminal session — hidden cursor, immediate key delivery, no echoed input, and stable frame output without tearing.

**How:** Each call builds on the previous. `initscr()` creates `stdscr` and puts the tty into raw mode. `noecho()` stops typed characters appearing on screen. `cbreak()` delivers keys immediately without Enter. `curs_set(0)` hides the blinking cursor. `nodelay(TRUE)` makes `getch()` return instantly when no key is waiting. `keypad(TRUE)` translates arrow-key escape sequences into integer constants. `typeahead(-1)` prevents ncurses from interrupting its output mid-write to poll stdin. Getting the order wrong crashes or silently breaks one of these properties.

Every file in this project boots ncurses in the same order inside `screen_init()`:

```c
initscr();               /* capture terminal into raw mode; create stdscr   */
noecho();                /* do not print typed chars to terminal             */
cbreak();                /* deliver keystrokes immediately (no Enter needed) */
curs_set(0);             /* hide the blinking cursor                         */
nodelay(stdscr, TRUE);   /* make getch() non-blocking                        */
keypad(stdscr, TRUE);    /* translate arrow/function keys to KEY_* constants */
typeahead(-1);           /* disable mid-flush stdin poll — atomic writes     */
start_color();           /* enable color pair support                        */
/* optional: use_default_colors() — see V1.3 */
getmaxyx(stdscr, rows, cols);  /* query terminal dimensions */
```

Order matters:
- `initscr()` must be first — it creates `stdscr` and all other calls depend on it.
- `start_color()` must precede any `init_pair()` call.
- `nodelay()` must precede the main loop or the first `getch()` will block.

*Files: all files*

---

#### V1.2 `endwin()` and `atexit` Cleanup

**What it is:** A cleanup hook that restores the terminal to its original state when the program exits by any means.

**What we achieve:** When the animation exits — whether normally, via `Ctrl+C`, or from `exit()` called deep inside the code — the shell is left in a working state with echo enabled, the cursor visible, and raw mode off. Without this, the terminal becomes unusable after the program ends.

**How:** We register a one-line cleanup function with `atexit()`. The C runtime calls all `atexit` functions automatically when `exit()` is called or when `main()` returns. Signal handlers set `running = 0` and let the main loop exit cleanly through `main()`, which then fires `atexit`. The combination of `atexit` + `SIGINT` handler + `SIGTERM` handler covers every normal exit path.

```c
static void cleanup(void) { endwin(); }
// ...
atexit(cleanup);
```

`endwin()` restores the terminal: re-enables echo, un-hides the cursor, leaves raw mode. Without it, a crash or `Ctrl+C` leaves the shell unusable.

`atexit()` guarantees `endwin()` runs even if:
- The program falls off `main()`.
- `exit()` is called from anywhere.
- A signal handler calls `exit()` via `running = 0` path.

It does NOT run on `SIGKILL`, `abort()`, or hardware faults — but those are unrecoverable anyway.

*Files: all files*

---

#### V1.3 `use_default_colors()` — Transparent Background

**What it is:** An opt-in extension that unlocks `-1` as a valid background color in `init_pair`, meaning "use whatever the terminal's own background is."

**What we achieve:** Characters float over the terminal's native background — whether that's a solid color, a wallpaper image, or a blur effect — instead of forcing a solid ncurses-controlled fill on every cell.

**How:** After `start_color()`, call `use_default_colors()` once. This registers the ISO 6429 SGR 49 escape code (reset background to default) as the `-1` value. Now `init_pair(1, 130, -1)` means "foreground colour 130, transparent background." Files that want a solid black background deliberately omit this call and use `COLOR_BLACK` explicitly.

```c
start_color();
use_default_colors();         /* must immediately follow start_color() */
init_pair(1, 130, -1);        /* -1 = terminal's default background color */
```

By default, ncurses requires both foreground and background to be explicit colors. Passing `-1` as the background color means "use whatever the terminal's background is" — allowing transparency in terminal emulators with image or blur backgrounds.

Without `use_default_colors()`, `-1` is invalid and `init_pair` silently uses `COLOR_BLACK`.

**Used in:**
- `bonsai.c` — tree branches draw over whatever is behind them.
- `matrix_rain.c` — rain characters over a transparent background.

**Not used in most files** because solid `COLOR_BLACK` background is intentional for fire, raster, and physics demos.

*Files: `bonsai.c`, `matrix_rain.c`*

---

#### V1.4 `cbreak()` vs `raw()`

**What it is:** Two modes that make keys available immediately (without waiting for Enter), but differ in how they handle control sequences like `Ctrl+C`.

**What we achieve:** With `cbreak()`, pressing `Ctrl+C` still generates `SIGINT`, letting the signal handler set `running = 0` for a clean, orderly exit. With `raw()`, `Ctrl+C` arrives as raw character 3 — no signal, no cleanup path unless you handle character 3 yourself.

**How:** `cbreak()` disables line buffering (so keys arrive immediately) but leaves signal processing intact. `raw()` disables both line buffering and signal processing — every keystroke, including control codes, goes directly to the application. All animation files use `cbreak()`. Only `aspect_ratio.c` uses `raw()` as a deliberate learning example.

Both modes deliver keypresses immediately without waiting for Enter. The difference:

| Mode | Ctrl+C | Ctrl+Z | Ctrl+\ |
|---|---|---|---|
| `cbreak()` | sends `SIGINT` | sends `SIGTSTP` | sends `SIGQUIT` |
| `raw()` | delivered as character 3 | delivered as character 26 | delivered as character 28 |

All animation files use `cbreak()` — they want `Ctrl+C` to trigger `SIGINT → on_exit_signal → running=0` for a clean exit. The `aspect_ratio.c` basic example uses `raw()` instead (no signal handling needed there).

*Files: `cbreak()` in all animation files; `raw()` in `aspect_ratio.c`*

---

#### V1.5 Monochrome Mode — Omitting `start_color()`

**What it is:** Running ncurses without calling `start_color()` at all, producing a program that uses only the default terminal foreground and background.

**What we achieve:** The absolute minimal render setup — no color registration, no pair management, no attribute logic. Every `mvaddch` writes in the terminal's default color.

**How:** Simply omit `start_color()` from the init sequence. `COLOR_PAIR`, `init_pair`, `attron` with color attributes — none of these are called. `wireframe.c` uses this: its entire visual effect comes from choosing the right slope character (`/`, `\`, `-`, `|`) per Bresenham step. No color adds anything to that effect, so it's cleanly absent.

```c
/* wireframe.c init — no start_color(), no color pairs */
initscr();
noecho();
cbreak();
curs_set(0);
nodelay(stdscr, TRUE);
keypad(stdscr, TRUE);
typeahead(-1);
/* start_color() deliberately omitted */
```

**When to use:** Any animation whose effect is entirely shape/character-based and gains nothing from color. Forces simplicity.

*Files: `wireframe.c`*

---

### V2 — Double Buffer Mechanics

#### V2.1 `curscr` / `newscr` — The Built-in Buffer Pair

**What it is:** ncurses' automatic double-buffer — two internal virtual screens that are always present, whether you know about them or not.

**What we achieve:** Only the cells that actually changed between frames are transmitted to the terminal. This eliminates full-screen flicker and keeps terminal I/O volume proportional to animation activity rather than screen size.

**How:** Every `mvaddch`, `attron`, and `erase` call writes into `newscr` — the frame being built this tick. Nothing reaches the terminal. When `doupdate()` is called, ncurses computes the diff `newscr − curscr`, sends only the changed cells as minimal escape codes, then updates `curscr = newscr`. You cannot opt out of this mechanism — it is created by `initscr()` and exists for the entire session.

ncurses maintains two virtual screens internally at all times:

```
curscr   — what ncurses believes the physical terminal currently shows
newscr   — the frame being constructed this render step
```

Every `mvwaddch`, `wattron`, `erase`, `mvprintw` call writes into `newscr`. Nothing reaches the terminal until `doupdate()` is called. `doupdate()` computes the diff `newscr − curscr`, sends the minimal set of escape codes, then updates `curscr = newscr`.

This is always present. You cannot opt out of it. The buffer pair is not created by the programmer — it exists from `initscr()` onwards.

*Files: all files*

---

#### V2.2 `erase()` vs `clear()` — Never Use `clear()` in a Loop

**What it is:** Two calls that both "blank the screen," but with very different consequences for how much data is sent to the terminal each frame.

**What we achieve:** With `erase()`, the diff engine still works — only genuinely changed cells are transmitted. With `clear()`, every single cell is marked as changed and the entire screen is retransmitted every frame, doubling I/O and causing full-screen flicker at 60fps.

**How:** `erase()` resets ncurses' internal `newscr` buffer to spaces — it writes nothing to the terminal itself. The diff engine then sees only cells that changed from the previous frame. `clear()` does the same reset but also schedules a `\e[2J` (clear screen) escape on the next `doupdate()`, forcing a full repaint. Always use `erase()` in animation loops.

| Call | What it does | Terminal I/O |
|---|---|---|
| `erase()` | Fills `newscr` with spaces (clears internal buffer) | None |
| `clear()` | Same + schedules `\e[2J` on next `doupdate()` | Full repaint every frame |
| `wclear(w)` | Same as `clear()` for window `w` | Full repaint |
| `werase(w)` | Same as `erase()` for window `w` | None |

`clear()` marks every cell as changed so `doupdate()` retransmits the entire screen. This doubles the terminal write volume and causes full-screen flicker at 60 fps.

`erase()` only resets the internal buffer. The diff engine will only transmit cells that actually changed — which for a typical animation frame is a small fraction of the terminal.

**Always use `erase()` in the render loop.**

*Files: all files*

---

#### V2.3 `wnoutrefresh` + `doupdate` — The Two-Phase Flush

**What it is:** The two-step process that moves content from ncurses' internal buffer to the actual terminal. Staging and committing are separated into distinct calls.

**What we achieve:** A single, atomic terminal write per frame. With the two-step form the intent is explicit and the code is ready for multi-window compositing without changing the flush logic.

**How:** `wnoutrefresh(stdscr)` stages the window's content into ncurses' internal `newscr` — still no terminal I/O. `doupdate()` then computes `newscr − curscr` and sends the diff in one write. The combined `wrefresh(w)` = `wnoutrefresh(w)` + `doupdate()` — functionally equivalent for a single window, but the split form is used throughout this project for clarity.

```c
/* screen_present() — one call per frame */
wnoutrefresh(stdscr);   /* stage stdscr → ncurses' internal newscr */
doupdate();             /* diff newscr vs curscr → terminal fd      */
```

- `wrefresh(w)` is `wnoutrefresh(w)` + `doupdate()` combined.
- With a single `stdscr` both are equivalent in result.
- Using the explicit two-call form makes the intent clear and enables future multi-window compositing without changing the flush code.

The whole project always uses the split form to document that the two phases are distinct.

*Files: all files*

---

#### V2.4 Why Manual Front/Back Windows Break the Diff Engine

**What it is:** An explanation of why the "two-window double buffer" pattern — a common beginner approach — produces ghost trails and tearing instead of clean animation.

**What we achieve (by avoiding this):** The diff engine works correctly, transmitting only genuinely changed cells. Ghost trails and the visual noise from spurious overwrites disappear.

**How the bug works:** When you copy a "back" window into a "front" window and refresh the front, ncurses compares the front window against `curscr` — not against your back window. Every cell of the front now looks "changed" even if the content didn't change, so the entire screen is retransmitted. The correct model: write everything directly into `stdscr`. ncurses' own `curscr/newscr` pair is the double buffer — you do not need to build one yourself.

A common ncurses animation mistake:

```c
/* WRONG — do not do this */
WINDOW *front = newwin(rows, cols, 0, 0);
WINDOW *back  = newwin(rows, cols, 0, 0);

// draw into back ...
overwrite(back, front);   /* or copywin */
wrefresh(front);
```

The problem: ncurses does not know `back` exists as a "working" surface. When you copy `back → front`, the diff engine sees spurious changes on every cell (it compares `front` as modified vs `curscr` as previously displayed). It retransmits far more cells than changed, producing ghost trails and tearing.

**The correct model:** there is exactly one visible window — `stdscr`. Write everything directly into it. ncurses' `curscr/newscr` is the double buffer you need.

The `aspect_ratio.c` basic example does use `newwin()` as a learning exercise, but all animation files use the single-`stdscr` model exclusively.

*Files: all animation files (correct); `aspect_ratio.c` (illustrative exception)*

---

#### V2.5 Framebuffer-to-ncurses: `cbuf` → `fb_blit()`

**What it is:** An intermediate plain-C array (`cbuf[]`) that the 3D rasteriser pipeline writes into instead of calling ncurses directly. A single `fb_blit()` function at the end of the frame transfers it to ncurses.

**What we achieve:** Complete separation between rendering math and terminal I/O. The rasteriser, z-test, and shader code can run as pure arithmetic with no ncurses state entangled in the loops. All ncurses calls are batched into one clean pass at the boundary.

**How:** Each pipeline stage (vertex shader → rasterise → z-test → fragment shader) writes `Cell {ch, color_pair, bold}` entries into `cbuf[cols*rows]` and float depth values into `zbuf[]`. After the full frame is built, `fb_blit()` iterates `cbuf`, skips cells where `ch == 0` (empty — not covered by any triangle), and calls `attron` + `mvaddch` + `attroff` for each visible cell. The entire ncurses API surface is just those three calls, in one function.

All raster files use an intermediate CPU framebuffer rather than writing directly to ncurses during rasterization:

```c
typedef struct { char ch; int color_pair; bool bold; } Cell;

Cell   cbuf[cols * rows];   /* filled by pipeline_draw_mesh() */
float  zbuf[cols * rows];   /* depth buffer */

/* After full frame is rasterized: */
static void fb_blit(const Framebuffer *fb)
{
    for (int y = 0; y < fb->rows; y++) {
        for (int x = 0; x < fb->cols; x++) {
            Cell c = fb->cbuf[y * fb->cols + x];
            if (!c.ch) continue;              /* skip empty cells */
            attr_t a = COLOR_PAIR(c.color_pair) | (c.bold ? A_BOLD : 0);
            attron(a);
            mvaddch(y, x, (chtype)(unsigned char)c.ch);
            attroff(a);
        }
    }
}
```

**Why the intermediate buffer:**
- The z-test, barycentric interpolation, and shader math can run without any ncurses calls.
- `fb_blit()` is the single well-defined boundary between rendering math and terminal I/O.
- Empty cells (ch == 0) are skipped — only actual geometry reaches the terminal.
- The pipeline stays pure C arithmetic; ncurses state changes are batched and localized.

*Files: all raster files (`torus_raster.c`, `cube_raster.c`, `sphere_raster.c`, `displace_raster.c`)*

---

#### V2.6 Diff-based Selective Clearing — No `erase()`

**What it is:** An alternative to `erase()` that clears only the cells that were non-empty last frame and are now empty — rather than wiping the entire screen buffer unconditionally.

**What we achieve:** Minimum terminal write volume. For sparse animations (fire that covers ~30% of the screen), a full `erase()` marks every cell as changed even if 70% of them were already empty. Selective clearing writes only the cells that actually transitioned from visible to empty.

**How:** Maintain a `prev[rows*cols]` snapshot copied from the heat buffer each frame. At the start of each frame, for every cell where `prev[i] > 0` and the new value is `0`, call `mvaddch(y, x, ' ')` to explicitly blank that cell. Draw all non-zero cells normally. After drawing, `memcpy(prev, bmap, ...)` snapshots the new state for next frame.

```c
/* aafire_port.c — diff-based clear instead of erase() */
for (int i = 0; i < rows * cols; i++) {
    if (prev[i] > 0 && bmap[i] == 0) {
        mvaddch(i / cols, i % cols, ' ');   /* blank only changed-to-empty cells */
    }
}
/* draw non-zero cells normally, then: */
memcpy(prev, bmap, (size_t)(rows * cols) * sizeof *bmap);
```

**Trade-off:** More bookkeeping (one extra buffer, one extra loop) in exchange for fewer escape codes sent per frame. Worth it when the animation covers a small fraction of the terminal. Not worth it for animations that fill the whole screen every frame.

*Files: `aafire_port.c`*

---

### V3 — Color System

#### V3.1 Color Pairs — `init_pair` / `COLOR_PAIR`

**What it is:** ncurses' color system. You cannot set a foreground color alone — you must register a numbered pair of (foreground, background) colors first, then activate that pair number before drawing.

**What we achieve:** Named, reusable color combinations. By giving each color combination a number, draw code can just say `COLOR_PAIR(3)` without repeating the RGB values, and changing a theme only requires re-registering pair numbers once.

**How:** At startup, call `init_pair(n, fg_color, bg_color)` for each combination you need (pairs are numbered from 1). During drawing, `attron(COLOR_PAIR(n))` activates pair n; `mvaddch` writes in that color; `attroff(COLOR_PAIR(n))` restores default. The number of available pairs is `COLOR_PAIRS` — typically 256 on modern terminals.

ncurses does not allow setting foreground color alone. Every colored character requires a registered color pair:

```c
start_color();
init_pair(1, COLOR_RED, COLOR_BLACK);   /* pair 1: red on black */
init_pair(2, 196, COLOR_BLACK);          /* 256-color: bright red on black */

/* later: */
attron(COLOR_PAIR(1) | A_BOLD);
mvaddch(y, x, '@');
attroff(COLOR_PAIR(1) | A_BOLD);
```

- Pair numbers are 1-based (`init_pair(0, ...)` modifies the default pair, avoid it).
- The number of available pairs is `COLOR_PAIRS` (typically 256 on modern terminals).
- All projects here use pairs 1..N where N is the number of visual gradients needed.

*Files: all files*

---

#### V3.2 256-Color vs 8-Color Fallback Pattern

**What it is:** A runtime branch inside `color_init()` that registers rich xterm-256 palette indices when available, or falls back to the 8 named ANSI colors on limited terminals.

**What we achieve:** The same compiled binary works correctly on a modern `xterm-256color` terminal and a basic 8-color SSH session. The animation degrades gracefully — fewer gradient steps, but the same visual intent.

**How:** Check `COLORS >= 256` after `start_color()`. If true, register specific xterm-256 index numbers (196 for bright red, 208 for orange, etc.). If false, map to the closest named color (`COLOR_RED`, `COLOR_YELLOW`, etc.). The pair numbers used in draw code stay the same — only the init branch differs.

Every file uses the same pattern to support both terminal types:

```c
static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        /* Rich palette: specific xterm-256 indices */
        init_pair(1, 196, COLOR_BLACK);   /* bright red  */
        init_pair(2, 208, COLOR_BLACK);   /* orange      */
        init_pair(3, 226, COLOR_BLACK);   /* yellow      */
        /* ... */
    } else {
        /* 8-color fallback: approximate with named colors */
        init_pair(1, COLOR_RED,    COLOR_BLACK);
        init_pair(2, COLOR_RED,    COLOR_BLACK);   /* no orange in 8-color */
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);
        /* ... */
    }
}
```

The check is at runtime so the same binary runs in `xterm-256color`, `tmux` (when not configured for 256), and basic VT100. The 8-color path degrades gracefully — fewer gradient steps but the same visual intent.

*Files: all files*

---

#### V3.3 xterm-256 Palette Index Reference

**What it is:** A map of which specific xterm-256 color indices appear in this project and the reasoning behind each choice.

**What we achieve:** Consistent, readable colors across animations. Picking the wrong index — say, a low-luminance blue on a black background — produces characters that are nearly invisible. Knowing the palette avoids this.

**How:** The 256-color space has three regions: indices 0–15 are the 8+8 ANSI named colors; 16–231 are a 6×6×6 RGB cube (index = 16 + 36r + 6g + b, r/g/b ∈ 0–5); 232–255 are 24 grey steps from near-black to white. This project uses the cube for vivid hues and the grey ramp for luminance gradients in raymarchers.

Specific indices used across this project (all with `COLOR_BLACK` background):

| Index | Approximate Color | Used For |
|---|---|---|
| 196 | Bright red (#ff0000) | Fire hot end, raster warm colors |
| 208 | Orange (#ff8700) | Fire mid, warm raster |
| 226 | Bright yellow (#ffff00) | Fire tips, HUD, bonsai HUD, leader yellow |
| 46 | Pure green (#00ff00) | Raster cool mid, matrix flock |
| 51 | Cyan (#00ffff) | Raster cool end, flock leader cyan |
| 21 | Pure blue (#0000ff) | Raster cold end |
| 201 | Magenta (#ff00ff) | Raster complement |
| 33 | Dodger blue (#0087ff) | Flock electric blue (vivid on dark bg) |
| 130 | Dark brown | Bonsai trunk |
| 172 | Amber brown | Bonsai branches |
| 28 | Dark green | Bonsai dense leaves |
| 82 | Lime green | Bonsai light leaves |
| 235–255 | Grey ramp (dark→white) | Raymarcher/donut luminance gradient |

**Why Dodger Blue (33) over Violet (93) in flocking.c:**
Blue-purples sit in the low-luminance zone of the 256-color cube — on black backgrounds they read as nearly invisible. Dodger blue (33) sits in the middle of the blue ramp where luminance is highest, staying vivid at any terminal brightness setting.

*Files: various*

---

#### V3.4 `use_default_colors()` + Background `-1`

**What it is:** The color-pair companion to V1.3 — using `-1` as the background argument in `init_pair` to make character backgrounds transparent.

**What we achieve:** Each character's background cell shows whatever the terminal was already displaying — a custom color, image, or translucent blur — instead of a solid ncurses-controlled color block.

**How:** After calling `use_default_colors()`, the value `-1` becomes valid as the background in `init_pair`. It maps to the ISO 6429 SGR 49 "default background" escape code. Only `bonsai.c` and `matrix_rain.c` use this — fire, raster, and physics demos want a solid black background and omit `use_default_colors()` entirely.

```c
start_color();
use_default_colors();
init_pair(1, 130, -1);   /* foreground 130, transparent background */
```

The `-1` background argument to `init_pair` means "the terminal's own background" — achieved through the ISO 6429 SGR 49 escape code. This lets bonsai branches and matrix rain characters float over transparent terminal backgrounds (common with image-in-terminal or blur effects).

**When not to use it:** fire, raster, and raymarching demos intentionally want a solid black background so `COLOR_BLACK` is used explicitly — `use_default_colors()` is omitted.

*Files: `bonsai.c`, `matrix_rain.c`*

---

#### V3.5 Grey Ramp for Luminance Gradients

**What it is:** Eight ncurses color pairs registered to evenly-spaced grey shades from the xterm-256 grey ramp (indices 232–255), used as a luminance scale for 3D rendering.

**What we achieve:** Smooth apparent brightness gradients across lit 3D surfaces — dark shadows through mid tones to bright highlights — using only color pairs and no special graphics hardware.

**How:** The xterm-256 grey ramp has 24 steps (232 = nearly black, 255 = white). We pick every 3rd step (235, 238, 241, 244, 247, 250, 253, 255) to get 8 evenly spaced levels. Each gets one `init_pair`. When the shader computes a luminance value for a surface point, it maps it to a pair index 1–8 and draws the character in that grey. Combined with the ASCII density ramp, this gives two independent brightness axes per cell.

Raymarchers and the donut renderer map continuous luminance values to a sequence of grey pairs:

```c
/* raymarcher.c — xterm-256 grey ramp: 235, 238, 241, 244, 247, 250, 253, 255 */
/* (every 3rd grey level from dark to white — even spacing) */
init_pair(1, 235, COLOR_BLACK);
init_pair(2, 238, COLOR_BLACK);
/* ... */
init_pair(8, 255, COLOR_BLACK);
```

The 256-color terminal's grey ramp runs from index 232 (nearly black) to 255 (white). Picking every 3rd gives 8 evenly-spaced luminance steps. Combined with the Paul Bourke ASCII ramp, this produces two independent axes of "brightness" — one via character density, one via terminal grey shade — doubling the apparent luminance resolution.

8-color fallback: `A_DIM` for dark bands, base color for mid, `A_BOLD` for bright:

```c
if (COLORS < 256) {
    if (l < 3) a |= A_DIM;
    else if (l >= 6) a |= A_BOLD;
}
```

*Files: `raymarcher.c`, `raymarcher_cube.c`, `donut.c`, `raymarcher_primitives.c`*

---

#### V3.6 `attr_t` — Combining Pair and Attribute Flags

**What it is:** ncurses' attribute type — a single integer that encodes both the color pair number and any style flags (`A_BOLD`, `A_DIM`, `A_REVERSE`, etc.) combined by bitwise OR.

**What we achieve:** One-call attribute activation. Instead of `attron(COLOR_PAIR(n))` then `attron(A_BOLD)` separately, we build the combined value in a variable first, then pass it to `attron()` in a single call. Conditional attributes (e.g., bold only when `life > 0.65`) become a simple `if` before the draw.

**How:** `COLOR_PAIR(n)` returns the pair encoded in the high bits of `attr_t`. `A_BOLD` and other flags occupy separate bit positions. Bitwise OR combines them: `attr_t a = COLOR_PAIR(3) | A_BOLD`. Pass `a` to `attron(a)` and `attroff(a)`. The exact bit layout is internal to ncurses — always use the macros, never hardcode bit values.

```c
attr_t attr = COLOR_PAIR(p->hue);
if (p->life > 0.65f) attr |= A_BOLD;
```

`attr_t` is ncurses' attribute type. A combined attribute is built by bitwise OR of `COLOR_PAIR(n)` with zero or more attribute flags (`A_BOLD`, `A_DIM`, `A_REVERSE`, etc.). The combined value is passed to `attron()` / `wattron()` in one call rather than multiple calls.

Building the attr into a variable before the draw call keeps the rendering code clean and allows conditional attribute logic (e.g., `A_BOLD` only when `life > 0.65`).

*Files: `brust.c`, `aafire_port.c`, `constellation.c`*

---

#### V3.7 A_BOLD / A_DIM as Brightness Tiers

**What it is:** Using `A_BOLD` and `A_DIM` not for font weight but as brightness modifiers — a feature of almost all terminal emulators where these flags shift the foreground color lighter or darker.

**What we achieve:** Three visible brightness levels from a single color pair at zero extra cost: `A_DIM` (faded/old), base (normal), `A_BOLD` (bright/hot). This triples the apparent gradient resolution on both 8-color and 256-color terminals.

**How:** Apply `A_BOLD` or `A_DIM` to the `attr_t` based on a simulation property — particle life, distance to leader, heat level, etc. For example: `if (p->life > 0.6) attr |= A_BOLD; else if (p->life < 0.2) attr |= A_DIM;`. The terminal emulator maps `A_BOLD` to a brighter variant of the foreground color and `A_DIM` to a dimmer one. Most terminal emulators honor both flags; `A_DIM` falls back gracefully on those that don't.

On both 8-color and 256-color terminals:
- `A_BOLD` makes the foreground color brighter (not actually bold font in most terminal emulators).
- `A_DIM` makes it dimmer (not always supported — falls back gracefully).
- `A_NORMAL` is the baseline.

This gives three brightness tiers per color pair at zero extra cost — useful for gradients when `COLORS < 256`.

**aafire_port.c** uses this most aggressively: it stores per-ramp-step `attr_t attr8[]` arrays for each theme with explicit `A_DIM`, `A_NORMAL`, `A_BOLD` sequences to construct a 9-step brightness gradient with only 8 base colors.

**brust.c** uses `A_BOLD` when `particle->life > 0.65f` (young, hot particles are brighter) and `A_DIM` for the decay scorch marks.

**flocking.c** uses `A_BOLD` for followers within 35% of perception radius from their leader, `A_NORMAL` otherwise — a distance-based brightness halo effect.

*Files: all files*

---

#### V3.8 Dynamic Color Update — Cosine Palette (flocking.c)

**What it is:** Calling `init_pair()` during the animation loop — not just at startup — to continuously re-register color pairs with new colors, animating the palette itself rather than just the characters.

**What we achieve:** Flock colors smoothly cycle through the full hue spectrum over time. The boids change color continuously without any change to how they are drawn — only the registered color behind `COLOR_PAIR(n)` changes.

**How:** A cosine formula generates RGB values that sweep through [0,1] smoothly: `r = 0.5 + 0.5*cos(2π*(t/period + phase_r))`. The float is mapped to the xterm-256 6×6×6 cube: `cube_idx = 16 + 36*ri + 6*gi + bi` where each channel is quantized to 0–5. `init_pair(pair_num, cube_idx, COLOR_BLACK)` is called every N frames. ncurses takes effect on the next `doupdate()`. Different RGB phase offsets per flock give each flock a different color trajectory.

`flocking.c` rotates flock colors over time by periodically updating the ncurses color pair registrations:

```c
/* Cosine palette: c = 0.5 + 0.5*cos(2π(t/period + phase)) */
float r = 0.5f + 0.5f * cosf(2.0f * M_PI * (t / period + phase_r));
float g = 0.5f + 0.5f * cosf(2.0f * M_PI * (t / period + phase_g));
float b = 0.5f + 0.5f * cosf(2.0f * M_PI * (t / period + phase_b));

/* Map float [0,1] → xterm-256 cube index (6×6×6 starting at 16) */
int ri = (int)(r * 5.0f + 0.5f);
int gi = (int)(g * 5.0f + 0.5f);
int bi = (int)(b * 5.0f + 0.5f);
int cube_idx = 16 + 36 * ri + 6 * gi + bi;

init_pair(pair_num, cube_idx, COLOR_BLACK);   /* re-register pair mid-animation */
```

The xterm-256 color cube occupies indices 16–231: `16 + 36r + 6g + b` where r,g,b ∈ [0,5]. Calling `init_pair()` during the animation loop is allowed — it takes effect on the next `doupdate()`. The cosine formula guarantees all RGB values stay in [0,1] and produces smoothly cycling, perceptually balanced hues.

*Files: `flocking.c`*

---

#### V3.9 Shade Enum → Composite `attr_t` Gradient

**What it is:** Mapping a small enum of named brightness levels to a combined `attr_t` value in a single lookup function, so draw code uses readable level names instead of raw attribute arithmetic.

**What we achieve:** A multi-level brightness gradient (e.g. six distinct visual intensities) with clean, readable draw code. The entire gradient is defined in one place; draw code just calls `shade_attr(level)`.

**How:** Define an enum with named levels (`FADE`, `DARK`, `MID`, `BRIGHT`, `HOT`, `HEAD`). Write `shade_attr(Shade s)` that returns the appropriate `COLOR_PAIR(n) | attr_flag`. The function handles the A_DIM/base/A_BOLD split internally. In the render loop, just call `shade_attr(grid[r][c])` — no attribute arithmetic at the call site.

```c
/* matrix_rain.c */
typedef enum { SHADE_FADE, SHADE_DARK, SHADE_MID,
               SHADE_BRIGHT, SHADE_HOT, SHADE_HEAD } Shade;

static attr_t shade_attr(Shade s) {
    switch (s) {
        case SHADE_FADE:   return COLOR_PAIR(1) | A_DIM;
        case SHADE_DARK:   return COLOR_PAIR(2);
        case SHADE_MID:    return COLOR_PAIR(3);
        case SHADE_BRIGHT: return COLOR_PAIR(4) | A_BOLD;
        case SHADE_HOT:    return COLOR_PAIR(5) | A_BOLD;
        case SHADE_HEAD:   return COLOR_PAIR(6) | A_BOLD;
    }
    return COLOR_PAIR(1);
}
```

**How to apply:** Use when you have 4+ brightness levels and want the draw code to stay readable. One enum change updates the entire gradient.

*Files: `matrix_rain.c`*

---

#### V3.10 Encapsulated Theme Struct

**What it is:** Bundling all color indices and attributes for one visual style into a struct, so switching themes is a single pointer swap rather than scattered `init_pair` calls throughout the code.

**What we achieve:** Clean theme switching — six fire palettes (fire, ice, plasma, nova, poison, gold) each with 9 ramp levels, all switchable at runtime with no draw-code changes. New themes are added by adding a struct literal; nothing else changes.

**How:** Define a struct with arrays for every per-level property. Write a `theme_apply(idx)` function that iterates the struct and calls `init_pair()` for each level. Draw code references pair numbers that never change — only the colors behind them change.

```c
/* fire.c */
typedef struct {
    int fg256[RAMP_N];   /* xterm-256 index per ramp level */
    int fg8[RAMP_N];     /* 8-color fallback per level */
    int attr8[RAMP_N];   /* A_DIM/0/A_BOLD per level for 8-color */
} FireTheme;

static const FireTheme k_themes[6] = {
    { {232,52,88,124,160,196,202,214,231}, ... },  /* fire  */
    { {232,17,19,21,27,33,51,123,231},     ... },  /* ice   */
    /* ... */
};

static void theme_apply(int t) {
    const FireTheme *th = &k_themes[t];
    for (int i = 0; i < RAMP_N; i++)
        init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
}
```

**How to apply:** Any animation with multiple color palettes. Keep all palette data in the struct; keep all pair numbers as constants; only `theme_apply()` touches `init_pair()`.

*Files: `fire.c`, `aafire_port.c`*

---

#### V3.11 Life-gated `A_BOLD` / `A_DIM`

**What it is:** Driving a particle's brightness attribute directly from its remaining lifetime — bold when freshly spawned, normal in mid-life, dim as it approaches death.

**What we achieve:** Particles that visually "burn out" — they flare brightly at spawn, settle to normal brightness, then fade to a dim ember before disappearing. Three apparent brightness states from one color pair, zero extra pairs.

**How:** Evaluate the particle's `life` float (1.0 = fresh, 0.0 = dead) against two thresholds before the draw call. OR the appropriate flag into the pair attribute.

```c
/* fireworks.c — two-threshold full lifecycle */
attr_t attr = COLOR_PAIR(p->color);
if      (p->life > 0.6f) attr |= A_BOLD;
else if (p->life < 0.2f) attr |= A_DIM;
/* between 0.2 and 0.6: base brightness */
attron(attr);
mvaddch(py, px, (chtype)(unsigned char)p->ch);
attroff(attr);

/* brust.c — single-threshold simpler lifecycle */
attr_t attr = COLOR_PAIR(p->color);
if (p->life > 0.65f) attr |= A_BOLD;
/* dead particles handled by scorch[] system with A_DIM separately */
```

**How to apply:** Use two thresholds for a smooth three-stage lifecycle (fireworks). Use one threshold when the "dead" stage is handled by a separate system (brust.c scorch marks). The scorch system draws dead footprints with `A_DIM` independently, so the live particles don't need the low-life dim stage.

*Files: `fireworks.c`, `brust.c`*

---

#### V3.12 Role-named Color IDs

**What it is:** Defining color pair identifiers by their semantic role in the scene rather than by number — `COL_BLOB_FAR`, `COL_BLOB_NEAR`, `COL_FLASH`, `COL_HUD` instead of `COLOR_PAIR(1)` through `COLOR_PAIR(6)`.

**What we achieve:** Draw code that reads like a description of the scene. `attron(COLOR_PAIR(COL_FLASH))` tells you what is being drawn; `attron(COLOR_PAIR(4))` does not. Adding a new role is an enum addition; renumbering pairs never breaks draw code.

**How:** Define an enum of role names. Map each role to a pair number in `color_init()`. Use the enum constants everywhere in draw code. Never use raw pair numbers after initialization.

```c
/* kaboom.c */
typedef enum {
    COL_BLOB_F = 1,  /* far blob — faded    */
    COL_BLOB_M,      /* mid blob            */
    COL_BLOB_N,      /* near blob — intense */
    COL_FLASH,       /* initial flash       */
    COL_RING,        /* wave rings          */
    COL_HUD,         /* HUD — always yellow */
} ColorID;

/* color_init() — HUD pair registered unconditionally outside theme block */
init_pair(COL_HUD, 226, COLOR_BLACK);   /* yellow, never changes with theme */
```

**How to apply:** Especially valuable in files with multiple rendering layers (flash, waves, blob, HUD) that each need distinct visual treatment. The HUD-always-one-color pattern (registered outside the theme block) is a direct consequence: the HUD pair is excluded from theme cycling so it stays readable regardless of blast color.

*Files: `kaboom.c`*

---

### V4 — Character Output

#### V4.1 `mvwaddch` — The Core Write Call

**What it is:** The fundamental ncurses call that writes one character to a specific position in a window. Every visible character in every animation ultimately reaches the screen through this call (or `mvaddch`, its `stdscr` shorthand).

**What we achieve:** Writing a single character at an exact (row, col) position with the current active attribute. Nothing is actually sent to the terminal yet — the character goes into `newscr`, and `doupdate()` later transmits it.

**How:** `mvwaddch(window, row, col, ch)` — note the argument order is `(y, x)`, row before column, the opposite of most graphics APIs. `mvaddch(row, col, ch)` is the shorthand that always targets `stdscr`. The character must be double-cast `(chtype)(unsigned char)ch` to prevent sign-extension bugs (see V4.2). Call `attron()` before and `attroff()` after to control its color and style.

```c
mvwaddch(w, row, col, (chtype)(unsigned char)ch);
```

`mvwaddch(window, y, x, char)` — note the `y, x` order (row first, then column), not `x, y`. Every coordinate in ncurses is (row, col) = (y, x). Getting this backwards produces mirrored/transposed scenes.

`mvwaddch` writes to a specific window. `mvaddch` (no `w`) writes to `stdscr`. Both are used in this project — `w`-variants appear in scene draw functions that accept a `WINDOW*` parameter; plain variants appear in functions that always use `stdscr`.

*Files: all files*

---

#### V4.2 `(chtype)(unsigned char)` Double Cast

**What it is:** A mandatory two-step type cast applied to every character before passing it to `mvaddch` — `(chtype)(unsigned char)ch` — that prevents two distinct corruption bugs caused by C's implicit integer promotion rules.

**What we achieve:** The character value stays in the range 0–255 and never accidentally activates ncurses attribute flag bits. Without this, high-value ASCII characters (128–255) produce garbage output: wrong colors, wrong characters, or random attribute activation.

**How:** `char` is signed on most platforms. A value like `'\xAF'` (175) has its sign bit set. When it's implicitly widened to `chtype` (an `unsigned long`), C sign-extends it to a large negative-looking value — for example `0xFFFFFFAF`. The upper bytes of `chtype` are ncurses attribute bits, so this accidentally activates whatever attributes those bits represent. The fix: `(unsigned char)ch` first zero-extends the value to 8 bits (0x00...00AF), then `(chtype)` widens it safely. One cast alone is not sufficient.

```c
mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
```

This double cast appears throughout the project and prevents two distinct bugs:

**Bug 1 — Sign extension:** `char` is signed on most platforms. High-value ASCII characters (128–255) have their sign bit set. When implicitly converted to `chtype` (an `unsigned long`), they sign-extend to large negative-looking values that ncurses interprets as attribute flags, printing garbage or random colors instead of the intended character.

**Fix:** Cast to `unsigned char` first — this zero-extends to 8 bits. Then cast to `chtype` — the value stays in [0, 255].

**Bug 2 — ACS confusion:** ncurses embeds attribute bits in the upper bytes of `chtype`. An uncast `char` with bit 7 set could accidentally activate these attribute bits. The unsigned cast isolates just the character bits.

*Files: all files that draw user-chosen characters*

---

#### V4.3 `wattron` / `wattroff` vs `attron` / `attroff`

**What it is:** Two families of attribute activation calls — `w`-prefixed variants for named `WINDOW*` objects, and plain variants that always target `stdscr`. They do the same thing; the difference is which window's attribute state they modify.

**What we achieve:** Colored and styled characters. Attributes must be activated before each character write and deactivated after — ncurses carries attribute state forward, so a missing `attroff` makes all subsequent characters in the frame inherit the attribute.

**How:** `attron(COLOR_PAIR(n) | A_BOLD)` activates the pair and bold flag on `stdscr`. `wattron(w, ...)` does the same on window `w`. In this project, scene draw functions that accept `WINDOW *w` use the `w`-forms; HUD functions that always write to `stdscr` use the plain forms. Every `attron` must be paired with a matching `attroff` using the same argument — otherwise the attribute leaks into subsequent draws.

```c
/* Window-specific (for scene_draw functions that accept WINDOW*) */
wattron(w, COLOR_PAIR(b->color) | A_BOLD);
mvwaddch(w, cy, cx, ch);
wattroff(w, COLOR_PAIR(b->color) | A_BOLD);

/* stdscr-specific (for screen_draw HUD functions) */
attron(COLOR_PAIR(3) | A_BOLD);
mvprintw(0, hx, "%s", buf);
attroff(COLOR_PAIR(3) | A_BOLD);
```

The `w`-prefixed versions apply to any `WINDOW*`; the unprefixed versions always target `stdscr`. In this project, scene draw functions receive `WINDOW *w` and use `wattron/wattroff`; HUD functions called from `screen_draw` use `attron/attroff`. Both write into `newscr` via ncurses' internal update chain.

**Important:** Always pair every `attron` with a matching `attroff`. Ncurses carries attribute state forward — a missing `attroff` leaves `A_BOLD` active for all subsequent writes in the frame, causing entire rows or scenes to appear incorrectly bright.

*Files: all files*

---

#### V4.4 `mvprintw` for HUD Text

**What it is:** The ncurses equivalent of `printf` that also positions the cursor first — it moves to (row, col) and writes formatted text in one call.

**What we achieve:** A fixed HUD overlay — FPS counter, mode name, key hints — positioned at a specific corner of the screen, drawn after all scene content so it always appears on top.

**How:** Format the HUD string into a fixed-size buffer with `snprintf` first (never write directly with format args that could contain user data). Then call `mvprintw(row, col, "%s", buf)` with `attron`/`attroff` brackets. The HUD is always written last in `screen_draw()` so it overwrites any scene character at the same cell position. Right-align to `cols - HUD_COLS` so it adapts to terminal width.

```c
char buf[HUD_COLS + 1];
snprintf(buf, sizeof buf,
         "%5.1f fps  balls:%-2d  %s  spd:%d",
         fps, sc->n, sc->paused ? "PAUSED " : "running", sim_fps);
int hud_x = s->cols - HUD_COLS;
if (hud_x < 0) hud_x = 0;
attron(COLOR_PAIR(3) | A_BOLD);
mvprintw(0, hud_x, "%s", buf);
attroff(COLOR_PAIR(3) | A_BOLD);
```

The HUD always uses:
1. `snprintf` into a fixed buffer (size-bounded, no overflow).
2. `"%s"` format string — the buffer content, not user data, so no format-string injection.
3. Written to row 0, right-aligned (or row `rows-1` for bottom HUD in bonsai.c).
4. Called **after** `scene_draw` so the HUD overwrites any scene character at the same cell.

*Files: all files*

---

#### V4.5 ACS Line-Drawing Characters

**What it is:** A set of ncurses macros (`ACS_ULCORNER`, `ACS_HLINE`, `ACS_VLINE`, etc.) that map to the terminal's own line-drawing character set at runtime — clean box-drawing characters without hardcoding Unicode.

**What we achieve:** Portable, visually clean borders and box outlines. On modern UTF-8 terminals these render as Unicode box-drawing characters (┌ ─ ┐ │ └ ┘). On legacy VT100 terminals they use the alternate character set. Plain ASCII (`+`, `-`, `|`) looks crude by comparison.

**How:** Use `ACS_*` macros directly in `mvaddch` — they expand to the correct terminal-specific `chtype` value. `bonsai.c` builds its message panel border by drawing the four corners, then filling the top and bottom edges with `ACS_HLINE` in a loop, and drawing left/right edges with `ACS_VLINE`. No Unicode literals, no terminal detection needed.

ncurses provides terminal-portable box-drawing characters through the `ACS_*` macros. Used in `bonsai.c` for the message panel box:

```c
attron(COLOR_PAIR(6) | A_BOLD);
mvaddch(by,         bx,             ACS_ULCORNER);   /* ┌ */
for (int i = 1; i < box_w-1; i++)
    mvaddch(by,     bx+i,           ACS_HLINE);      /* ─ */
mvaddch(by,         bx + box_w - 1, ACS_URCORNER);   /* ┐ */

mvaddch(by+1,       bx,             ACS_VLINE);      /* │ */
mvaddch(by+1,       bx + box_w - 1, ACS_VLINE);      /* │ */

mvaddch(by+2,       bx,             ACS_LLCORNER);   /* └ */
for (int i = 1; i < box_w-1; i++)
    mvaddch(by+2,   bx+i,           ACS_HLINE);      /* ─ */
mvaddch(by+2,       bx + box_w - 1, ACS_LRCORNER);   /* ┘ */
attroff(COLOR_PAIR(6) | A_BOLD);
```

`ACS_*` macros translate to the correct terminal-specific values at runtime — on VT100 terminals they use the alternate character set; on UTF-8 terminals they use Unicode box-drawing. Always prefer `ACS_*` over hardcoded Unicode or ASCII approximations for maximum terminal compatibility.

Available symbols used here: `ACS_ULCORNER`, `ACS_URCORNER`, `ACS_LLCORNER`, `ACS_LRCORNER`, `ACS_HLINE`, `ACS_VLINE`.

*Files: `bonsai.c`*

---

#### V4.6 Paul Bourke ASCII Density Ramp

**What it is:** A 92-character string where characters are ordered from visually lightest (space, nearly invisible) to visually heaviest (`@`, maximum ink coverage). Mapping a float luminance value to an index in this string gives an ASCII "greyscale pixel."

**What we achieve:** Brightness variation encoded purely through character choice. A fire flame, a 3D lit surface, or a raymarched sphere can show shading and gradients using only ASCII characters — no color required, though color is added on top for richer results.

**How:** Given a luminance `luma ∈ [0.0, 1.0]`, compute `idx = (int)(luma * 91)` and draw `k_bourke[idx]`. The string is ordered so low indices are sparse (light, few pixels lit) and high indices are dense (dark, many pixels lit). Combined with the grey-ramp color pairs, this gives two independent brightness axes: character density and color shade.

```c
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
```

92 characters ordered from visually sparse (space) to visually dense (`@`). Mapping `luma ∈ [0,1]` to an index gives an ASCII "pixel density" that encodes brightness. Used in all raster files, all raymarchers, and fire renderers.

The ramp is split into the dual-axis technique:
- **Character density** (sparse vs dense glyphs) — the primary brightness axis.
- **Color pair** (grey ramp 235..255 or warm-to-cool palette) — the secondary axis.

Together they provide ~700 distinguishable brightness levels in a single terminal cell.

*Files: all raster files, all raymarcher files, `fire.c`, `aafire_port.c`*

---

#### V4.7 Directional Characters — Velocity to Glyph

**What it is:** Choosing the character to draw based on the direction of an object's velocity, so the glyph itself points where the object is moving.

**What we achieve:** At a glance you can see which way every boid, arrow, or particle is heading. The simulation state is visible in the character itself, not just its position.

**How:** Compute the heading angle with `atan2(vy, vx)`. Quantize it into octants by dividing by π/4. Look up the corresponding character in a small array. For ncurses, negate `vy` first because row 0 is at the top (y increases downward), opposite to the mathematical convention where y increases upward. Unicode arrow glyphs (`→↗↑…`) are used in `flowfield.c` via `addwstr`; ASCII approximations (`>v<^`) are used in `flocking.c` for portability.

Two files map velocity direction to a glyph that visually indicates the direction of motion:

**flowfield.c — 8-way arrows:**
```c
static const char *k_dirs[8] = { "→", "↗", "↑", "↖", "←", "↙", "↓", "↘" };
int octant = (int)((atan2f(-vy, vx) + M_PI) / (M_PI / 4.0f)) % 8;
char *ch = k_dirs[octant];
```

`atan2(vy, vx)` returns the angle in `[-π, π]`. Adding `π` shifts to `[0, 2π]`, dividing by `π/4` gives the octant index 0–7. The negated `vy` accounts for ncurses' inverted Y axis (row 0 is top).

**flocking.c — per-flock velocity characters:**
```c
/* Each flock has a distinct character set so flocks remain visually distinct
   even when boids overlap in the same screen region */
static const char k_boid_chars[3][8] = {
    { '>', 'v', '<', '^', '>', 'v', '<', '^' },   /* flock 0: arrows */
    /* ... */
};
```

**spring_pendulum.c — spring segment slope:**
```c
/* Bresenham line: choose char based on step direction each iteration */
if      (step_x && step_y)  ch = (sx == sy) ? '\\' : '/';
else if (step_x)             ch = '-';
else                         ch = '|';
```

*Files: `flowfield.c`, `flocking.c`, `spring_pendulum.c`*

---

#### V4.8 Branch/Slope Characters — Angle to `/`, `\`, `|`, `-`

**What it is:** Selecting the best-fitting ASCII character for each step of a line based on the local slope, so that diagonal, horizontal, and vertical segments look distinct and physically plausible.

**What we achieve:** Lines that look like real objects — wire, a spring coil, tree branches — rather than a sequence of identical characters. A diagonal drawn with all `-` looks wrong; drawn with a mix of `-`, `\`, `/`, and `|` it reads as a physical line.

**How:** During Bresenham line traversal, track the step direction at each cell (step in X only, Y only, or both diagonals). Map each case to a character: X-only → `-`, Y-only → `|`, both with same sign → `\`, both with opposite sign → `/`. The threshold between "mostly horizontal" and "diagonal" is tuned by comparing `abs(dx)` against `abs(dy)/2`.

`spring_pendulum.c` and `bonsai.c` use slope-to-character mapping for drawing lines that look like wire or branches:

```c
/* bonsai.c branch character selection based on dx/dy direction */
if      (abs(dx) < abs(dy) / 2)  ch = '|';    /* mostly vertical   */
else if (abs(dy) < abs(dx) / 2)  ch = '-';    /* mostly horizontal */
else if (dx * dy > 0)             ch = '\\';   /* positive slope    */
else                              ch = '/';    /* negative slope    */
```

This makes drawn lines look like physical structures (wire, wood) rather than sequences of a single character. The threshold ratios (here `abs(dx) < abs(dy)/2`) determine the angular range where each character is preferred — widening the threshold makes the diagonal characters appear more, narrowing it makes `|` and `-` dominate.

*Files: `spring_pendulum.c`, `bonsai.c`, `wireframe.c`*

---

#### V4.9 Unicode Multi-byte Glyphs via `addwstr`

**What it is:** Writing multi-byte UTF-8 characters (Unicode arrows, box glyphs, etc.) to the terminal by using `addwstr`/`mvaddwstr` instead of `mvaddch`, which only handles single-byte characters.

**What we achieve:** Visually precise direction indicators — `→ ↗ ↑ ↖ ← ↙ ↓ ↘` genuinely point the way particles are flowing. ASCII approximations (`> ^ < v`) work but look cruder.

**How:** Store direction strings as `const wchar_t *dirs[8]`. Move to position with `move(y, x)` then call `addwstr(dirs[octant])`. The terminal must be in a UTF-8 locale for multi-byte sequences to render correctly — `setlocale(LC_ALL, "")` at program start enables this. `mvadd_wch` or `mvaddwstr` handle the sequence without manual byte splitting.

```c
/* flowfield.c */
static const char *k_dirs[8] = { "→","↗","↑","↖","←","↙","↓","↘" };

int octant = (int)((atan2f(-vy, vx) + (float)M_PI) / ((float)M_PI / 4.0f)) % 8;
move(draw_row, draw_col);
addstr(k_dirs[octant]);   /* addstr handles UTF-8 byte sequences */
```

**How to apply:** Use when the direction itself is the primary visual information (flow fields, vector visualizations). Fall back to ASCII arrow chars (`>v<^`) if the terminal locale is uncertain or for simpler needs (flocking.c uses ASCII per-flock char sets for portability).

*Files: `flowfield.c`*

---

### V5 — Visual Effects Techniques

#### V5.1 Draw Order / Z-Ordering — Last Write Wins

**What it is:** The terminal's implicit depth model — there is no z-buffer or alpha blending. When two objects occupy the same cell, whichever one calls `mvaddch` last is what appears.

**What we achieve:** Correct visual layering: background behind foreground, foreground behind HUD. By controlling draw order we control which elements are always visible and which can be hidden behind others.

**How:** Draw everything in order from back to front. Background elements first, physics objects second, HUD last. The HUD written at the end of `screen_draw()` will always overwrite any scene character at the same position. `spring_pendulum.c` documents its 6-layer draw order explicitly: support bar → wire stubs → coil lines → coil nodes (overwrite lines) → bob (overwrite nodes).

ncurses cells are overwritten in place — there is no blending, transparency, or compositing. The last `mvaddch` call to a given (row, col) wins. This is the only form of "depth sorting" available in a terminal.

Convention in this project:
1. Background elements are drawn first.
2. Foreground / interactive elements drawn second.
3. HUD drawn last (always visible, cannot be obscured).

`spring_pendulum.c` documents its exact draw order in comments:
```
1. Top bar (row 0, full width)
2. Wire stub: pivot → first coil node
3. Spring coil connecting lines between adjacent nodes
4. Wire stub: last coil node → bob
5. Spring coil nodes (overwrite connecting lines with '*')
6. Iron bob '(@)' (overwrite any wire at bob position)
```

*Files: all files*

---

#### V5.2 HUD Always On Top

**What it is:** A draw-order guarantee: the HUD is always written last in the frame, ensuring it overwrites any scene content at the same cell position.

**What we achieve:** The FPS counter, mode display, and key hints remain readable at all times, regardless of what the animation draws in the same screen region.

**How:** In `screen_draw()`, call `scene_draw()` first (fills `newscr` with animation content), then write the HUD text. Since last write wins, the HUD characters are what actually appear. The HUD is placed at row 0 right-aligned, a position that most animations leave mostly empty. `bonsai.c` additionally writes a second HUD row at `rows - 1`.

The HUD (FPS counter, mode display, key hints) must always be readable regardless of what the animation draws. The pattern:

```c
static void screen_draw(...)
{
    erase();                    /* clear newscr */
    scene_draw(...);            /* draw animation into newscr */
    /* HUD: written LAST — overwrites any scene character at same cell */
    attron(COLOR_PAIR(hud_pair) | A_BOLD);
    mvprintw(0, hud_x, "%s", hud_buf);
    attroff(COLOR_PAIR(hud_pair) | A_BOLD);
}
```

Row 0, column `cols - HUD_COLS` is a standard HUD position — top-right corner, just wide enough for the formatted string. The `snprintf` target is always `HUD_COLS + 1` bytes so the string is bounded even if terminal is very narrow.

`bonsai.c` writes a second HUD line at `rows - 1` (bottom of screen) for additional status.

*Files: all files*

---

#### V5.3 Two-Pass Rendering (matrix_rain.c)

**What it is:** Splitting a single frame's draw into two sequential passes that serve different visual purposes — a persistence/fade layer and a motion/interpolation layer — because no single pass can do both correctly.

**What we achieve:** Matrix rain where the fading trail texture decays organically and the leading column head moves with sub-cell smoothness. Neither is possible with a single-pass approach.

**How:** Pass 1 iterates the `grid[row][col]` array — a persistent simulation-level texture that tracks each cell's fade state. Cells that are fading are drawn with their current shade attribute. Pass 2 draws each active column's head character at a float position computed from `head_y + speed * alpha` (render interpolation). The head's row is rounded to the nearest cell but the fractional position drives the interpolation, giving the appearance of motion between cells.

`matrix_rain.c` draws each frame in two distinct passes:

**Pass 1 — grid texture:**
```c
/* Iterate grid[row][col] — persistent dissolve state */
for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
        if (grid[r][c] != 0) {
            attron(pair_for(grid[r][c]));
            mvaddch(r, c, glyph_for(grid[r][c]));
            attroff(...);
        }
    }
}
```

**Pass 2 — live column heads (interpolated):**
```c
/* Each active column is drawn at float draw_head_y (render-interpolated) */
col_paint_interpolated(c, alpha, cols, rows);
```

The grid handles the organic "fade to black" tail texture via stochastic `grid_scatter_erase()`. The live column pass handles the sharp leading character at a sub-cell interpolated position. Neither pass alone is sufficient — pass 1 provides persistence, pass 2 provides smooth motion at the head.

*Files: `matrix_rain.c`*

---

#### V5.4 Stippled Bresenham Lines — Distance Fade (constellation.c)

**What it is:** Drawing only every Nth cell along a Bresenham line, leaving gaps between drawn cells to make the line appear sparser and therefore visually fainter.

**What we achieve:** Three apparent levels of connection strength between stars — bright solid lines for nearby pairs, normal solid lines for mid-range, dotted lines for distant pairs — without any actual opacity or blending support in the terminal.

**How:** Maintain a `step_count` integer during Bresenham traversal. Only call `mvaddch` when `step_count % stipple == 0`. `stipple = 1` draws every cell (solid line); `stipple = 2` draws every other cell (dotted). Combined with `A_BOLD` for close connections and normal for far, this gives six distinct visual states across just two parameters.

`constellation.c` draws connection lines between stars with visual "distance fade" using stippling — drawing only every Nth cell along the Bresenham line:

```c
/* stipple = 1: draw every cell (bright, close connections)
   stipple = 2: draw every 2nd cell (dimmer, far connections) */
int step_count = 0;
while (drawing_line) {
    if (step_count % stipple == 0) {
        if (!cell_used[y][x]) {
            wattron(w, attr);
            mvwaddch(w, y, x, '-');   /* or '|', '/' based on slope */
            wattroff(w, attr);
        }
    }
    step_count++;
    /* advance Bresenham */
}
```

Combined with attribute selection:
- `ratio < 0.50` → `A_BOLD`, `stipple = 1` (bright solid line)
- `ratio < 0.75` → normal, `stipple = 1` (medium solid line)
- `ratio < 1.00` → normal, `stipple = 2` (faint dotted line)

This gives three visual "bands" of connection strength using only two parameters.

*Files: `constellation.c`*

---

#### V5.5 `cell_used[][]` Grid — Preventing Line Overdraw

**What it is:** A per-frame boolean grid that tracks which screen cells have already been claimed by a line this frame, preventing subsequent lines from overwriting them.

**What we achieve:** Clean-looking line intersections. Without this, every Bresenham step overwrites the previous line's character at shared cells, producing a confusing tangle of collision characters at intersection points.

**How:** Declare `bool cell_used[rows][cols]` on the stack (a VLA — `rows` and `cols` are runtime values) and `memset` it to zero at the start of each frame's line drawing. In the Bresenham loop, before calling `mvaddch`, check `if (!cell_used[y][x])`. If the cell is free, draw and set `cell_used[y][x] = true`. If already claimed, skip it. The first line to reach a cell wins — its character is preserved cleanly.

When multiple connection lines pass through the same screen region, they would overwrite each other's characters every Bresenham step, producing a visually noisy "knot" of mixed characters.

`constellation.c` prevents this with a per-frame `bool cell_used[rows][cols]`:

```c
bool cell_used[rows][cols];
memset(cell_used, 0, sizeof cell_used);

/* In draw_line: */
if (!cell_used[y][x]) {
    mvwaddch(w, y, x, ch);
    cell_used[y][x] = true;   /* claim this cell; subsequent lines skip it */
}
```

The array is stack-allocated each frame (VLA — `rows` and `cols` are runtime values). The first line to reach a cell claims it; all other lines skip it. The result: dense connection clusters show smooth individual lines rather than a jumbled character pile.

*Files: `constellation.c`*

---

#### V5.6 Proximity Brightness — `A_BOLD` by Distance (flocking.c)

**What it is:** Using `A_BOLD` as a brightness halo around a flock leader — followers near the leader draw bold (brighter), followers farther away draw normal.

**What we achieve:** A visible "attraction" effect. The leader appears surrounded by a glowing cluster that thins out toward the edges of the flock. This conveys the social structure of the simulation without any additional geometry or color pairs.

**How:** Compute the toroidal shortest-path distance between follower and leader (toroidal because the simulation wraps at screen edges — a follower near the right edge and a leader near the left edge are actually close). Divide by `PERCEPTION_RADIUS` to get a ratio 0–1. If ratio < 0.35, return `A_BOLD`; otherwise `A_NORMAL`. OR this into the attribute before drawing. The toroidal distance calculation is essential — without it, cross-edge proximity produces incorrect dim results.

Followers in the same flock glow brighter when they are near their leader:

```c
static int follower_brightness(const Boid *follower, const Boid *leader,
                                float max_px, float max_py)
{
    /* Toroidal shortest-path distance — correct even across wrap edges */
    float dx    = toroidal_delta(follower->px, leader->px, max_px);
    float dy    = toroidal_delta(follower->py, leader->py, max_py);
    float ratio = hypotf(dx, dy) / PERCEPTION_RADIUS;
    return (ratio < 0.35f) ? A_BOLD : A_NORMAL;
}
```

The return value is used directly as the attribute modifier: `COLOR_PAIR(n) | follower_brightness(...)`. This creates a visual "halo" of bright followers around the leader — a proximity-based visual depth cue with no extra geometry.

The toroidal distance is essential: without it, followers that are physically close but have crossed the wrap boundary would incorrectly appear dim.

*Files: `flocking.c`*

---

#### V5.7 Bayer 4×4 Ordered Dithering → ASCII Density

**What it is:** Adding a small, spatially-varying offset to a luminance value before quantizing it to an ASCII character, so that the rounding error at each quantization step forms a regular halftone pattern rather than a flat banding artefact.

**What we achieve:** Smooth apparent gradients across 3D surfaces. Without dithering, a surface that varies from 0.4 to 0.5 luminance would snap to the same ASCII character — appearing as a flat band. With Bayer dithering it shows a fine 4×4 dot pattern that the eye reads as a smooth gradient.

**How:** The Bayer matrix is a fixed 4×4 array of threshold values ranging from 0 to 15/16, arranged so their spatial distribution is maximally even. Before looking up the ASCII character, add `(bayer[py % 4][px % 4] - 0.5) * amplitude` to `luma`, clamp to [0,1], then index the Bourke ramp. The amplitude (0.15 in this project) is tuned: too large gives a noisy dotted look, too small gives banding.

Before mapping luminance to an ASCII character, a position-dependent threshold is added:

```c
static const float k_bayer[4][4] = {
    {  0/16.0f,  8/16.0f,  2/16.0f, 10/16.0f },
    { 12/16.0f,  4/16.0f, 14/16.0f,  6/16.0f },
    {  3/16.0f, 11/16.0f,  1/16.0f,  9/16.0f },
    { 15/16.0f,  7/16.0f, 13/16.0f,  5/16.0f },
};

float dithered = luma + (k_bayer[py & 3][px & 3] - 0.5f) * 0.15f;
dithered = fmaxf(0.0f, fminf(1.0f, dithered));
int idx = (int)(dithered * (BOURKE_LEN - 1));
```

The `& 3` (modulo 4) selects the matrix entry by cell position. The dither amplitude `0.15` is tuned: too high gives a noisy dotted pattern, too low gives flat quantization steps. The result: gradients render as spatial density patterns (like halftone printing) rather than abrupt luminance jumps.

*Files: all raster files, `fire.c`*

---

#### V5.8 Floyd-Steinberg Error Diffusion → ASCII (fire.c)

**What it is:** A dithering algorithm that tracks the rounding error from each quantized cell and distributes it to unprocessed neighboring cells using weighted fractions (7/16, 3/16, 5/16, 1/16), spreading the error naturally through the image.

**What we achieve:** Smooth, organic-looking heat gradients without a regular repeating pattern. Flame tongues and heat pools look naturally shaped. The error "flows" along the gradient direction, producing results that feel physically continuous rather than spatially patterned like Bayer dithering.

**How:** Process the heat grid top-to-bottom, left-to-right. For each cell, quantize to the nearest ASCII ramp level, compute the error (original − quantized), and add weighted fractions of that error to the right, lower-left, below, and lower-right neighbors (the classical Floyd-Steinberg 4-neighbor kernel). Work on a copy of the heat array so modifications don't corrupt cells that haven't been processed yet. More expensive than Bayer (requires a working copy), but produces more organic results for flame-shaped data.

`fire.c` and `aafire_port.c` use Floyd-Steinberg error diffusion instead of ordered dithering for smoother heat gradients:

```c
/* Process top-to-bottom, left-to-right */
for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
        float old_val  = heat_work[y][x];
        float quant    = quantize(old_val);    /* snap to nearest ramp level */
        float err      = old_val - quant;

        /* Distribute error to 4 unprocessed neighbours */
        if (x + 1 < cols)           heat_work[y][x+1]   += err * 7.0f/16.0f;
        if (y + 1 < rows) {
            if (x > 0)              heat_work[y+1][x-1] += err * 3.0f/16.0f;
                                    heat_work[y+1][x]   += err * 5.0f/16.0f;
            if (x + 1 < cols)       heat_work[y+1][x+1] += err * 1.0f/16.0f;
        }
        /* draw quant value at (y, x) */
    }
}
```

The 7-3-5-1 weight distribution "spends" rounding error across adjacent cells, producing smoother gradients with no regular pattern. Better than ordered dithering for organic shapes like flames — worse for performance (requires a working copy of the heat grid to avoid modifying values in-flight).

*Files: `fire.c`, `aafire_port.c`*

---

#### V5.9 Luminance-to-Color Mapping

**What it is:** Using the same luminance value that picks an ASCII character to also pick a color pair from a warm-to-cool sequence, so bright areas appear warm (red/yellow) and shadow areas appear cool (blue/magenta).

**What we achieve:** Approximate real-world light source color temperature with minimal code. A lit 3D surface feels more physically believable when highlights are warm and shadows are cool, even on an ASCII terminal.

**How:** After computing `dithered_luma`, multiply by the number of color pairs and clamp: `cp = clamp(1 + (int)(luma * 6), 1, 7)`. Pairs 1–3 are warm (red, orange, yellow), pairs 4–5 are neutral (green), pairs 6–7 are cool (blue, magenta). High luminance → low pair index (warm); low luminance → high pair index (cool). The same `luma` is separately used to index the Bourke ramp for the character.

Raster files map the computed luminance value to both a character AND a color pair simultaneously, using luminance to select the warm-to-cool color progression:

```c
/* luma_to_cell — raster files */
int  cp  = 1 + (int)(d * 6.0f);   /* d = dithered luma [0,1] → pair 1..7 */
if (cp > 7) cp = 7;
bool bold = d > 0.6f;              /* bright areas get A_BOLD */
return (Cell){ ch, cp, bold };
```

Color pair mapping: `1`=bright red (warm), `4`=green (neutral), `7`=magenta (cool complement). Bright objects appear warm/yellow; shadow areas appear cool/blue. This approximates real-world light source color temperature with minimal code.

*Files: all raster files*

---

#### V5.10 Scorch Mark Accumulation (brust.c)

**What it is:** A simulation-level persistence array that stores the footprint of past explosions and redraws them every frame as dimmed residue below the live particles.

**What we achieve:** A screen that accumulates a history of all past explosions. Each new burst leaves a faded mark that remains visible through subsequent bursts, creating a sense of continuity and physical consequence.

**How:** On each explosion, append the affected cell positions and characters to a `scorch[]` array. Every frame, before drawing active particles, iterate `scorch[]` and draw each entry with `COLOR_PAIR(C_ORANGE) | A_DIM`. Since `erase()` clears `newscr` each frame, scorch must be actively redrawn every frame — the persistence lives in the simulation array, not in ncurses state. `A_DIM` makes it visibly fainter than the live particles drawn on top.

`brust.c` maintains a persistent `scorch[]` array that survives across burst cycles — past explosions leave marks on the ground:

```c
/* Scorch cells drawn with A_DIM to appear as faded residue */
wattron(w, COLOR_PAIR(C_ORANGE) | A_DIM);
for (int i = 0; i < b->n_scorch; i++) {
    int y = b->scorch_y[i];
    int x = b->scorch_x[i];
    if (y >= 0 && y < rows && x >= 0 && x < cols)
        mvwaddch(w, y, x, (chtype)(unsigned char)b->scorch[i]);
}
wattroff(w, COLOR_PAIR(C_ORANGE) | A_DIM);
```

`A_DIM` makes the scorch appear faded relative to active particles. New scorch entries are added on each explosion; old ones persist. This creates visual continuity between bursts — the screen accumulates a history of all past explosions without requiring image compositing.

*Files: `brust.c`*

---

#### V5.11 Flash Cross-pattern — Instant Impact Indicator

**What it is:** Drawing a `'*'` at an explosion center and `'+'` at the four cardinal neighbors simultaneously — five cells that form a cross shape — all in one bold color. The pattern lasts exactly one frame.

**What we achieve:** An unambiguous "hit here" signal that the eye catches instantly. One cross drawn with `A_BOLD` is more legible than a single character and creates the feeling of a physical impact without any particle simulation at the moment of detonation.

**How:** Draw the center first, then four offset writes. No loop needed — five `mvwaddch` calls. Apply once at the exact tick the explosion triggers, not in subsequent ticks.

```c
/* brust.c — flash cross drawn at explosion tick only */
wattron(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
mvwaddch(w, cy,   cx,   '*');   /* center */
mvwaddch(w, cy-1, cx,   '+');   /* north  */
mvwaddch(w, cy+1, cx,   '+');   /* south  */
mvwaddch(w, cy,   cx-1, '+');   /* west   */
mvwaddch(w, cy,   cx+1, '+');   /* east   */
wattroff(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
```

**How to apply:** Any explosion or impact event. Combine with a particle burst for full effect: cross on frame 0, particles from frame 1 onward.

*Files: `brust.c`*

---

#### V5.12 Z-depth → Character + Color Selection

**What it is:** Using a 3D point's projected depth value to simultaneously select both the ASCII character (sparse for far, dense for near) and the color pair (faded for far, intense for near), giving a single-pass depth-shading effect with no z-buffer.

**What we achieve:** A particle blob that reads as three-dimensional — distant elements look dotted and dim, close elements look heavy and bright. No depth sorting or z-buffer is needed because each element independently encodes its own depth.

**How:** Compute `bz` (the z-coordinate in projected space). Compare against two percentage thresholds of the perspective distance `persp`. Use a three-way branch to pick char and pair.

```c
/* kaboom.c — 3D blob element rendering */
float bz = blob_z_at(angle, radius);
char ch; int col;
if      (bz > persp * 0.8f) { ch = '.'; col = COL_BLOB_F; }  /* far   */
else if (bz > persp * 0.2f) { ch = 'o'; col = COL_BLOB_M; }  /* mid   */
else                         { ch = '@'; col = COL_BLOB_N; }  /* near  */
```

**How to apply:** Works for any particle system where elements span a z-range. The character ramp (`.` → `o` → `@`) leverages Bourke density intuition: sparse dots for distant, heavy symbols for close. Adjust the 0.8/0.2 thresholds to control how quickly depth is expressed.

*Files: `kaboom.c`*

---

#### V5.13 Dual-factor Visual Function

**What it is:** A single function that takes two orthogonal simulation properties (age AND neighbor count) and writes both the display character AND the attribute through output pointers — encoding two independent dimensions of information in one character cell.

**What we achieve:** A grain of sand that simultaneously communicates how old it is (via character) and how compacted it is (via brightness). No other code in the draw path handles the mapping; it is centralized and easy to tune.

**How:** The function accepts two inputs and two output pointers. Character selection uses one input; attribute selection uses the other. The call site is minimal — just pass both simulation values and receive the render spec.

```c
/* sand.c */
static void grain_visual(int age, int neighbor_count,
                          char *ch_out, attr_t *attr_out) {
    /* age controls character density: sparse when young, dense when settled */
    static const char k_levels[] = "`" ".oO0#";
    int ci = age < (int)(sizeof k_levels - 1) ? age : (int)(sizeof k_levels - 2);
    *ch_out  = k_levels[ci];
    /* neighbor_count controls brightness: isolated = bold, packed = normal */
    *attr_out = COLOR_PAIR(CP_GRAIN) | (neighbor_count < 2 ? A_BOLD : 0);
}

/* call site: */
char ch; attr_t attr;
grain_visual(g->age, g->neighbors, &ch, &attr);
attron(attr);
mvaddch(g->row, g->col, (chtype)(unsigned char)ch);
attroff(attr);
```

**How to apply:** Use whenever a simulation object has two independent visual dimensions worth encoding. Centralizing both mappings in one function means changing the visual representation requires touching only one place.

*Files: `sand.c`*

---

#### V5.14 Ring-buffer Trail Coloring

**What it is:** Storing a particle's last N positions in a ring buffer, then iterating from newest to oldest during draw — assigning progressively higher (dimmer) pair indices to older positions.

**What we achieve:** Trails that visually fade from bright (recent) to dim (old), encoding the particle's recent history as a color gradient behind it. The ring buffer ensures constant O(1) memory and O(N) draw per particle regardless of trail length.

**How:** Each particle keeps a `trail[]` array with a `head` index. Each tick, advance `head` and overwrite the oldest entry. In draw, iterate `i` from 0 to trail_length−1: position at `(head - i + N) % N` gets pair `1 + i` (lower index = newer = brighter).

```c
/* flowfield.c */
for (int i = 0; i < TRAIL_LEN; i++) {
    int ti = (p->head - i + TRAIL_LEN) % TRAIL_LEN;
    if (!p->trail_active[ti]) continue;
    attron(COLOR_PAIR(1 + i));   /* 1=newest/brightest, TRAIL_LEN=oldest/dimmest */
    mvaddch(p->trail[ti].row, p->trail[ti].col, '.');
    attroff(COLOR_PAIR(1 + i));
}
```

**How to apply:** Allocate N color pairs where pair 1 is the brightest and pair N is the dimmest (or use `A_DIM` for the oldest). Register them once at init. The ring buffer ensures the draw cost is exactly N calls per particle regardless of how long the trail has been running.

*Files: `flowfield.c`*

---

#### V5.15 `luma_to_cell` — Dither + Ramp + Color in One Call

**What it is:** A single function that converts a float luminance value into a complete `Cell {ch, color_pair, bold}` struct by chaining Bayer dithering → Bourke ramp lookup → warm-to-cool pair selection in sequence.

**What we achieve:** All luminance-to-display-output logic in one function. Every place in the pipeline that needs to convert a shading result to a drawable cell calls the same function — consistent, testable, easy to tune.

**How:** Apply the Bayer threshold offset, clamp, look up the Bourke character, compute the warm-to-cool pair index (high luma = warm = low index; low luma = cool = high index), set bold for bright fragments, pack into Cell.

```c
/* torus_raster.c / raster files */
static Cell luma_to_cell(float luma, int px, int py)
{
    /* 1. Bayer 4×4 ordered dither */
    float d = luma + (k_bayer[py & 3][px & 3] - 0.5f) * 0.15f;
    d = fmaxf(0.0f, fminf(1.0f, d));

    /* 2. Bourke ramp → character */
    char ch = k_bourke[(int)(d * (BOURKE_LEN - 1))];

    /* 3. Warm-to-cool pair: high luma → pair 1 (red/warm), low → pair 7 (cool) */
    int cp = 1 + (int)(d * 6.0f);
    if (cp > 7) cp = 7;

    /* 4. Bold for bright fragments */
    bool bold = d > 0.6f;

    return (Cell){ ch, cp, bold };
}
```

**How to apply:** Call from the fragment shader or rasterizer inner loop. Centralizing the conversion means changing dither strength, ramp length, or color mapping requires editing one function — the pipeline is untouched.

*Files: `torus_raster.c`, `cube_raster.c`, `sphere_raster.c`, `displace_raster.c`*

---

#### V5.16 Depth Sort — Painter's Algorithm

**What it is:** Computing all renderable points for a frame into a buffer, sorting them by depth (back to front), then drawing in that order so nearer points naturally overwrite farther ones.

**What we achieve:** Correct overlap without a z-buffer. Since each parametric point maps to a screen cell and only one character can occupy a cell, sorting ensures the front-most character always wins without per-cell depth comparison.

**How:** For each `(θ, φ)` sample: compute the 3D point, apply rotations, project to screen, compute luminance. Store `(cx, cy, depth, char, pair)` in a flat array. Sort descending by depth (farthest first). Iterate and call `mvaddch` — later writes (closer) overwrite earlier (farther).

```c
/* donut.c — parametric torus depth sort */
/* 1. Compute all points */
for each (theta, phi):
    compute (x, y, z) after rotation
    project to (cx, cy)
    points[n++] = { cx, cy, z, luma_char, pair };

/* 2. Sort farthest-first */
qsort(points, n, sizeof *points, cmp_depth_desc);

/* 3. Draw — closer points overwrite farther */
for (int i = 0; i < n; i++)
    mvaddch(points[i].cy, points[i].cx, points[i].ch);
```

**When to use vs z-buffer:** Depth sort is simpler to implement and works well for single continuous surfaces (torus, sphere) where the point count is small and each point maps to one unique cell. Z-buffer (zbuf[] array) is better for triangle rasterization where many triangles may cover the same cell and sorting all fragments is impractical.

*Files: `donut.c`*

---

### V6 — Input Handling

#### V6.1 `nodelay` — Non-blocking Input

**What it is:** A mode flag that makes `getch()` return `ERR` immediately when no key is pending, instead of blocking the thread until one arrives.

**What we achieve:** An animation loop that runs continuously at the target frame rate regardless of whether the user is pressing keys. Without this, the loop freezes every frame waiting for input.

**How:** Call `nodelay(stdscr, TRUE)` once in `screen_init()`. In the main loop, call `getch()` and check the return value: `ERR` means no key pending (continue the frame), any other value is a key code. Process the key if needed and move on. Only one key is drained per `getch()` call — if the user presses multiple keys quickly, they queue up and are processed one per frame.

```c
nodelay(stdscr, TRUE);
```

Without this, `getch()` blocks the thread until a key is pressed — the animation loop stalls. With `nodelay(TRUE)`, `getch()` returns `ERR` immediately when no key is pending.

The pattern in every main loop:
```c
int ch = getch();
if (ch != ERR && !app_handle_key(app, ch))
    app->running = 0;
```

`app_handle_key` returns `false` only for quit keys (`q`, `ESC`). All other keys modify state and return `true`. This pattern handles multiple simultaneous keys correctly — each `getch()` call drains one key from the input queue; the loop processes at most one key per frame.

*Files: all files*

---

#### V6.2 `keypad(stdscr, TRUE)` — Function and Arrow Keys

**What it is:** A mode that makes ncurses intercept multi-byte escape sequences from arrow keys and function keys, delivering them as single integer constants (`KEY_UP`, `KEY_LEFT`, `KEY_F(1)`, etc.) instead of raw escape bytes.

**What we achieve:** Simple, readable key handling. A single `switch(ch)` covers all keys — arrows, function keys, and regular characters — without needing to manually parse multi-byte sequences.

**How:** Arrow keys send `\e[A` (up), `\e[B` (down), etc. to the terminal. Without `keypad(TRUE)`, these arrive as three separate characters. `keypad(stdscr, TRUE)` tells ncurses to watch for these sequences and replace them with the `KEY_*` integer constants before delivering to `getch()`. The same `getch()` call handles both a letter press (`'q'`) and an arrow key (`KEY_UP`).

```c
keypad(stdscr, TRUE);
```

Without this, arrow keys and function keys arrive as multi-character escape sequences (e.g., `\e[A` for up-arrow). With `keypad(TRUE)`, ncurses intercepts these sequences and delivers single integer constants:
- `KEY_UP`, `KEY_DOWN`, `KEY_LEFT`, `KEY_RIGHT`
- `KEY_F(1)` through `KEY_F(12)`
- `KEY_RESIZE` — sent by some terminals on resize (project uses `SIGWINCH` instead)

*Files: all files*

---

#### V6.3 `typeahead(-1)` — Prevent Mid-flush Poll

**What it is:** A call that disables ncurses' built-in behavior of interrupting its output write to check for pending input mid-flush.

**What we achieve:** Tear-free frame output. The entire diff from `doupdate()` is written to the terminal in one uninterrupted burst. Without this, a busy frame (many cells changed) gets split into multiple partial writes with stdin polls in between, and the terminal shows a visually torn partial frame.

**How:** By default, `typeahead(fd)` tells ncurses to poll `fd` for input during output. `typeahead(-1)` disables the feature entirely — no polling at all during `doupdate()`. Input events are still collected; they're just handled on the next `getch()` call instead of mid-write. No input is lost — only the interrupt timing changes.

```c
typeahead(-1);
```

By default ncurses interrupts its output stream mid-flush to poll stdin for pending input. On fast terminals (or when many cells change), this fragments the `doupdate()` write into multiple small writes, causing visible tearing — the terminal shows a partial frame between flushes.

`typeahead(-1)` disables the poll: ncurses writes the entire diff atomically. Input is still checked on the next `getch()` call — no events are lost, just the interrupt.

*Files: all files*

---

#### V6.4 `noecho()` — Suppress Key Echo

**What it is:** A mode that stops the terminal from automatically printing typed characters to the screen as the user presses keys.

**What we achieve:** A clean animation display. Without `noecho()`, pressing `q` to quit would print the letter `q` into the animation, corrupting it. Every key press would leave a visible character on screen.

**How:** One call at startup: `noecho()`. From that point on, every keystroke goes silently into ncurses' input queue but is not echoed to the display. The program decides what to display — the terminal does not do it automatically.

```c
noecho();
```

Prevents typed characters from appearing in the terminal as the user presses keys. Without this, pressing `q` during the animation would print the letter `q` to the terminal, corrupting the display.

*Files: all files*

---

#### V6.5 `curs_set(0)` — Hide the Cursor

**What it is:** A call that hides the terminal cursor for the duration of the program.

**What we achieve:** A distraction-free animation. The terminal cursor is a blinking block that moves to the position of every `mvaddch` call. During a frame, `mvaddch` is called hundreds of times across arbitrary positions — the cursor visibly jumps around the entire screen. Hiding it eliminates this visual noise entirely.

**How:** `curs_set(0)` at startup hides the cursor. `curs_set(1)` would show it again, but that's not needed since `endwin()` on exit restores the terminal to its original cursor state automatically.

```c
curs_set(0);
```

Without this, the terminal cursor is left at the position of the last `mvaddch` call — it flickers visibly as it jumps around the screen every frame, distracting from the animation.

`curs_set(0)` hides it entirely. `curs_set(1)` would show it again on exit (not needed since `endwin()` restores terminal state).

*Files: all files*

---

### V7 — Signal Handling & Resize

#### V7.1 `SIGWINCH` — Terminal Resize Signal

**What it is:** A POSIX signal sent by the OS to the process whenever the terminal window is resized. Without handling it, the animation continues drawing to the old dimensions after the user resizes.

**What we achieve:** Correct behavior after a resize — the animation adapts to the new terminal dimensions within one frame, reallocating any size-dependent buffers and re-querying rows/cols.

**How:** Install a signal handler that only sets a `volatile sig_atomic_t need_resize = 1` flag — no ncurses calls in the handler since they're not async-signal-safe. In the main loop, check the flag at the top of each iteration. When set, clear it and call `screen_resize()` (the `endwin → refresh → getmaxyx` sequence). Reset `frame_time` and `sim_accum` after resizing to avoid a large `dt` spike from the resize latency.

The OS sends `SIGWINCH` when the terminal emulator window is resized. The project handles it with the flag pattern:

```c
/* Signal handler — only writes the flag */
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* Main loop — handles resize at safe point */
if (app->need_resize) {
    app->need_resize = 0;
    app_do_resize(app);
    frame_time = clock_ns();  /* reset so dt doesn't include resize stall */
    sim_accum  = 0;
}
```

The actual resize work (`endwin → refresh → getmaxyx → scene_reinit`) happens in the main loop body, not in the signal handler — ncurses functions are not async-signal-safe and must not be called from handlers.

`frame_time` and `sim_accum` are reset after resize to avoid a large `dt` spike from the resize latency.

*Files: all files*

---

#### V7.2 `volatile sig_atomic_t` — Signal-safe Flags

**What it is:** The only C type the standard guarantees can be safely written by a signal handler and read by the main thread without a data race or missed update.

**What we achieve:** Signal handlers that correctly communicate with the main loop. Without `volatile`, the compiler may cache `running` in a register and the main loop never sees the update. Without `sig_atomic_t`, the read/write might be non-atomic on some architectures, producing torn values.

**How:** Declare both `running` and `need_resize` as `volatile sig_atomic_t` in the `App` struct. The `volatile` keyword forces every read to go to memory — the optimizer cannot keep a cached copy in a register. `sig_atomic_t` guarantees atomic read/write from a signal handler context. In practice this is almost always `int`, but relying on that would be undefined behavior.

```c
typedef struct {
    /* ... */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;
```

`volatile` — prevents the compiler from caching the value in a register. Without `volatile`, the optimiser may move the loop condition check (`while (app->running)`) before the signal write, causing an infinite loop even after `Ctrl+C`.

`sig_atomic_t` — the only integer type the C standard guarantees can be read/written atomically from a signal handler without data races. On virtually all architectures this is `int`, but relying on that would be undefined behaviour per the standard.

*Files: all files*

---

#### V7.3 `endwin() → refresh() → getmaxyx()` Resize Sequence

**What it is:** The three-step sequence required to make ncurses pick up new terminal dimensions after a resize event. Each step is necessary; skipping any one causes the dimensions to remain stale.

**What we achieve:** After the user resizes the terminal, `rows` and `cols` reflect the actual new size within one frame. The animation then reallocates size-dependent buffers and draws correctly in the new dimensions.

**How:** `endwin()` puts the terminal back into normal (non-ncurses) mode, allowing the OS to update the tty's internal window size struct. `refresh()` re-enters ncurses mode, during which ncurses queries the OS for the current terminal size and updates its internal `LINES`/`COLS`. Only after this call does `getmaxyx(stdscr, rows, cols)` return the new values. Without `endwin() + refresh()`, `LINES` and `COLS` retain the pre-resize values indefinitely.

```c
static void screen_resize(Screen *s)
{
    endwin();                              /* exit ncurses mode temporarily  */
    refresh();                             /* re-enter: query new dimensions */
    getmaxyx(stdscr, s->rows, s->cols);   /* read updated terminal size     */
}
```

`endwin()` puts the terminal back into normal mode. `refresh()` re-initializes ncurses with the new terminal dimensions — it queries the OS for the current `LINES`/`COLS` values. Only after `refresh()` does `getmaxyx(stdscr, ...)` return the new size.

Without `endwin() + refresh()`, the stored `rows/cols` never update and the animation renders as though the terminal never changed size.

*Files: all files*

---

#### V7.4 `atexit(cleanup)` — Guaranteed Terminal Restore

**What it is:** Registering a cleanup function with the C runtime so it fires automatically on every normal exit path, guaranteeing `endwin()` is always called.

**What we achieve:** The shell is never left in a broken state after the animation exits. Without this, a crash or an `exit()` from deep inside the code would leave the terminal in raw/noecho mode — the shell prompt wouldn't echo input and `Ctrl+C` wouldn't work.

**How:** `atexit(cleanup)` registers a function pointer. The C runtime calls all registered `atexit` functions in reverse-registration order when `exit()` is called or when `main()` returns. Signal handlers set `running = 0` and let the main loop fall through `main()` normally, which triggers `atexit`. The three-line combination — `atexit` + `SIGINT` handler + `SIGTERM` handler — covers all realistic exit scenarios. `SIGKILL` cannot be caught, but terminal emulators reset the tty when the child process dies anyway.

```c
static void cleanup(void) { endwin(); }

int main(void) {
    atexit(cleanup);
    signal(SIGINT,  on_exit_signal);
    signal(SIGTERM, on_exit_signal);
    /* ... */
}
```

`atexit` fires when:
- `exit()` is called from anywhere.
- The program returns from `main()`.
- A signal handler sets `running=0` and the main loop exits normally.

It does NOT fire on `SIGKILL`, `abort()`, or hardware faults. But those leave the process dead anyway; the terminal emulator typically resets the tty on its own when the child process exits.

The three-line combination (`atexit` + `SIGINT` + `SIGTERM`) means `endwin()` runs in all normal exit paths — the terminal is never left in raw/noecho mode after the program exits.

*Files: all files*

---

### V8 — Performance & Correctness

#### V8.1 Sleep Before Render — Stable Frame Cap

**What it is:** Placing the frame-rate sleep budget at the beginning of the frame's I/O phase, before `doupdate()`, so that unpredictable terminal write time cannot destabilize the frame cap.

**What we achieve:** A stable 60fps cap regardless of how many cells changed this frame or how fast the terminal can process escape codes. The sleep absorbs any per-frame jitter from physics computation; terminal write runs in whatever time remains.

**How:** Measure `elapsed` = time spent on physics this tick. Sleep `16ms − elapsed` before any terminal I/O. Then call `screen_draw()` and `doupdate()`. Since the sleep is done, the terminal write runs without time pressure. If `doupdate()` takes 5ms on a busy frame, the next physics tick starts slightly late — but the cap remains stable because the sleep already happened. The naive opposite (sleep after `doupdate()`) must budget for `doupdate()` time, which varies per frame and produces erratic fps.

```c
/* CORRECT ORDER in the main loop */

/* 1. measure physics time (not including terminal I/O) */
int64_t elapsed = clock_ns() - frame_time + dt;

/* 2. sleep the remaining budget BEFORE any terminal I/O */
clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

/* 3. terminal I/O runs *after* the sleep */
screen_draw(...);     /* writes to newscr — no terminal I/O yet */
screen_present();     /* doupdate() — the single terminal write  */
getch();              /* input poll */
```

If the sleep happens AFTER `doupdate()`, the sleep budget must compensate for unpredictable terminal write time. On a slow terminal or when many cells change, `doupdate()` can take 5–10ms — pushing `elapsed` over the 16ms budget and making the sleep zero. The loop runs full-speed, frame rate becomes erratic.

By sleeping first (measuring only physics computation), the terminal write is "free time" that cannot destabilize the cap.

*Files: all files — the ordering is documented in Architecture.md as a critical correctness property*

---

#### V8.2 `getmaxyx` vs `LINES`/`COLS`

**What it is:** Two ways to read the terminal dimensions after `refresh()` — `getmaxyx(stdscr, rows, cols)` reads from the window struct directly; `LINES` and `COLS` are global variables that ncurses updates.

**What we achieve:** Accurate terminal dimensions at any time. Both give the same values for `stdscr` after a resize cycle, but `getmaxyx` is preferred because it explicitly names the window being queried and is portable if sub-windows are ever added.

**How:** After calling `endwin() + refresh()` during a resize, both `LINES`/`COLS` and `getmaxyx` will reflect the new size. In `tst_lines_cols.c` the globals are used for simplicity; all animation files use `getmaxyx` for clarity of intent.

```c
/* Preferred — window's actual current dimensions */
getmaxyx(stdscr, s->rows, s->cols);

/* Alternative — global ncurses vars, always reflect current terminal size */
s->rows = LINES;
s->cols = COLS;
```

Both are equivalent for `stdscr` after `refresh()`. This project always uses `getmaxyx` because:
- It reads the window's dimensions directly, not a global variable.
- It is portable to sub-windows if the code ever needs them.
- The intent (query this specific window) is more explicit.

`LINES`/`COLS` are used in the basic examples (`tst_lines_cols.c`) for simplicity.

*Files: `getmaxyx` in all animation files; `LINES/COLS` in `tst_lines_cols.c`*

---

#### V8.3 `CLOCK_MONOTONIC` — No NTP Jumps

**What it is:** A hardware-based clock that only moves forward and is completely unaffected by system clock adjustments (NTP sync, `date` command, timezone changes).

**What we achieve:** Stable `dt` values every frame. Physics simulations compute `dt = now − last_frame`. If the system clock jumps forward or back (NTP sync mid-animation, manual clock set), `dt` becomes huge and the simulation explodes — particles fly off screen, springs go infinite. `CLOCK_MONOTONIC` makes this impossible.

**How:** Use `clock_gettime(CLOCK_MONOTONIC, &t)` instead of `CLOCK_REALTIME` for all timing. Add a `dt` cap (`if (dt > 100ms) dt = 100ms`) as a second defense against the one case `CLOCK_MONOTONIC` cannot help with: the process being suspended by the OS (debugger, `SIGSTOP`, sleep) and resumed.

```c
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
```

`CLOCK_REALTIME` can jump forward (NTP sync) or backward (manual clock adjustment) mid-animation, producing a huge `dt` spike and physics explosion. `CLOCK_MONOTONIC` is a hardware counter that only moves forward and is unaffected by any clock adjustment. Every timing measurement in this project uses `CLOCK_MONOTONIC`.

The `dt` cap (`if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS`) handles the one case `CLOCK_MONOTONIC` cannot protect against: the process being suspended and resumed.

*Files: all files*

---

#### V8.4 Common ncurses Bugs and How This Project Avoids Them

**What it is:** A consolidated reference of the bugs you will reliably encounter when writing ncurses animations from scratch — each one a real failure mode that has been deliberately addressed by a specific technique in this project.

**What we achieve:** By recognizing each bug pattern, you can diagnose display corruption, freezes, and terminal corruption quickly and apply the corresponding fix rather than debugging the symptom.

**How:** Each row maps a visible symptom to its root cause and the specific technique this project uses to prevent it. Use this table as a diagnostic starting point when something looks wrong.

| Bug | Cause | Fix Applied |
|---|---|---|
| Ghost trails | Manual front/back window pair breaks diff engine | Single `stdscr` model everywhere |
| Full-screen flicker | Using `clear()` in render loop | Always use `erase()` |
| Animation freeze | Missing `nodelay(TRUE)` — `getch()` blocks | Set in `screen_init()` |
| Tearing (mid-frame) | `typeahead` enabled — doupdate split by stdin poll | `typeahead(-1)` in `screen_init()` |
| Cursor artifact | Cursor visible, jumps to last `mvaddch` position | `curs_set(0)` in `screen_init()` |
| Terminal corruption on crash | `endwin()` not called | `atexit(cleanup)` + signal handler chain |
| Physics explosion on resume | Large `dt` after suspend/debugger | Cap `dt` at 100ms |
| Wrong resize dimensions | No `endwin+refresh` before `getmaxyx` | `screen_resize()` does full cycle |
| Sign extension in `mvaddch` | `char` → `chtype` implicit sign extension | `(chtype)(unsigned char)ch` everywhere |
| Attribute leak | `attron` without matching `attroff` | All attron/attroff calls are paired |
| Color pair 0 modified | `init_pair(0, ...)` alters default pair | Pairs numbered from 1 |
| Blocked loop | Signal written to non-`sig_atomic_t` flag, compiler caches it | `volatile sig_atomic_t` |
| Banker's rounding flicker | `roundf()` for pixel→cell conversion at boundary | `floorf(x + 0.5f)` throughout |

---

---

## V9 — Per-File Technique Reference

Each entry lists the unique techniques used by that file. Techniques already documented in V1–V8 are referenced by section; file-specific patterns are explained inline.

---

### tst_lines_cols.c
*Minimal LINES/COLS demo — the simplest possible ncurses program.*

**LINES / COLS globals** — `initscr()` sets the globals `LINES` and `COLS` to terminal dimensions. Read them directly: `printw("rows=%d cols=%d", LINES, COLS)`. No function call needed. Animation files prefer `getmaxyx` (→ V8.2) for explicit window querying.

**`printw` / `refresh()`** — `printw` writes into `newscr` at the current cursor; `refresh()` = `wnoutrefresh(stdscr)` + `doupdate()` in one call. Simplest possible text output pattern.

**Blocking `getch()`** — without `nodelay(TRUE)`, `getch()` halts until a key is pressed. Used here intentionally to pause the program after printing. All animation files use `nodelay(TRUE)` (→ V6.1) to prevent this.

**No signal handling, no `start_color()`** — every line is essential, every safety mechanism is absent. Teaching baseline.

---

### aspect_ratio.c
*Demonstrates correct circle drawing by compensating for non-square terminal cells.*

**`newwin(rows, cols, 0, 0)`** — creates a `WINDOW*` separate from `stdscr`. All draw calls use `w`-prefixed forms (`wmove`, `waddch`, etc.). Animation files avoid this pattern (→ V2.4); it is used here as a teaching example.

**`wbkgd(win, COLOR_PAIR(1))`** — sets the background attribute for an entire window. Every cleared cell gets this attribute. Called once after `newwin`; subsequent `werase()` fills with it automatically.

**`werase(win)` / `wrefresh(win)`** — `werase` clears the window buffer; `wrefresh` = `wnoutrefresh(win)` + `doupdate()`. Animation files always use the split form (→ V2.3).

**`raw()` instead of `cbreak()`** (→ V1.4) — intentional: no signal handling, Ctrl+C arrives as character 3.

**Aspect x×2 correction** — circle drawn with `x = cx + r * 2 * cos(angle)`, `y = cy + r * sin(angle)`. Multiplying x by 2 compensates for cells being ~2× taller than wide, making the circle visually round.

---

### physics/bounce_ball.c
*Reference implementation — the canonical animation skeleton every other file follows.*

**7 spectral color pairs** — red(196), orange(208), yellow(226), green(46), cyan(51), blue(21), magenta(201). The canonical set reused across fireworks, constellation, brust, kaboom.

**`wattron / mvwaddch / wattroff` bracket** — attribute active only for one `mvwaddch` call; prevents color leaking. Reference example for the bracket pattern (→ V4.3).

**`(chtype)(unsigned char)ch` double cast** (→ V4.2) — documented with an inline comment explaining sign extension. Only file to explain it in-source.

**Pixel-space physics with `floorf` round-half-up** (→ Architecture.md §4) — `CELL_W=8`, `CELL_H=16`; `px_to_cell_x = floorf(px/CELL_W + 0.5f)`.

**Forward render interpolation (alpha)** (→ Architecture.md §4) — `draw_px = b->px + b->vx * alpha * dt_sec`.

---

### ncurses_basics/spring_pendulum.c
*Lagrangian spring-pendulum with Bresenham coil rendering and layered draw order.*

**Semantic color pair names** — `CP_BAR`, `CP_WIRE`, `CP_SPRING`, `CP_BALL`, `CP_HUD` constants instead of bare indices. `wattron(w, CP_SPRING)` conveys intent; `wattron(w, COLOR_PAIR(3))` does not.

**6-layer explicit draw order** (→ V5.1) — bar → wire stubs → coil lines → coil nodes → bob. Each layer overwrites the previous. Documented in source comments.

**Slope char from Bresenham step direction** (→ V4.8) — one ternary per step: `(sx&&sy) ? (sx==sy?'\\':'/') : sx?'-':'|'`.

**`prev_r`/`prev_theta` lerp for non-linear physics** (→ Architecture.md §4) — store prev before tick, lerp in draw: `draw_r = prev_r + (r - prev_r) * alpha`.

---

### matrix_rain/matrix_rain.c
*Matrix rain with 6-shade gradient, transparent background, and hot-swappable themes.*

**`use_default_colors()` + pair `-1` background** (→ V1.3, V3.4) — rain floats over terminal wallpaper.

**6-shade `Shade` enum → composite `attr_t`** (→ V3.9) — `FADE/DARK/MID/BRIGHT/HOT/HEAD` mapped by `shade_attr()`.

**Runtime theme swap** (→ V3.10 pattern) — `theme_apply(idx)` re-registers 6 pairs; draw code unchanged.

**Two-pass rendering** (→ V5.3) — Pass 1: persistent grid texture. Pass 2: interpolated float head positions.

**Float head with `floorf(y + 0.5f)`** — `draw_row = (int)floorf(draw_head_y + 0.5f)`. Deterministic cell assignment without banker's rounding oscillation.

---

### particle_systems/fire.c
*Doom-style fire CA with 9-level ASCII ramp, Floyd-Steinberg dithering, auto-cycling themes.*

**9-level ramp `" .:+x*X#@"`** — sparse (cold) to dense (hot). `heat → index 0–8 → ramp[index]`.

**`FireTheme` struct** (→ V3.10) — 6 themes (fire/ice/plasma/nova/poison/gold), each with `fg256[9]`, `fg8[9]`, `attr8[9]`.

**Auto-cycling themes** — `cycle_tick` increments each sim tick; at threshold, advances theme and calls `theme_apply()`.

**Floyd-Steinberg dithering** (→ V5.8) — on a scratch copy of the heat grid before ramp lookup.

**Per-cell `attron`/`attroff`** — each cell sets its own attribute independently; no global attribute state between cells.

**Non-uniform LUT breaks** — `{0.000, 0.080, 0.180, 0.290, 0.390, 0.500, 0.620, 0.750, 0.900}` — clustered in 0.3–0.75 where flame curvature is most perceptually important.

---

### particle_systems/aafire_port.c
*aalib fire variant — minimises terminal write volume using diff-based clearing.*

**Diff-based clearing** (→ V2.6) — no `erase()`; only cells that transition from visible to empty receive a space character.

**`memcpy(prev, bmap, ...)` snapshot** — copied after drawing, not before. Snapshot after ensures the diff compares the actual drawn state.

**5-neighbour CA** — `(y+1,x-1), (y+1,x), (y+1,x+1), (y+2,x-1), (y+2,x+1)` averaged. Produces rounder, slower-rising blobs vs. fire.c's 4-neighbour spikes.

**Per-row decay LUT** — `minus[y] = max_decay * y / rows`. Faster decay near top (flame tips die), slower at bottom (fuel zone). Self-normalizes to any terminal height.

---

### particle_systems/fireworks.c
*Rocket fireworks with three-state machine and life-gated brightness.*

**7 spectral pairs** — canonical set (196/208/226/46/51/21/201). Same 7 pairs reused in brust, kaboom, constellation.

**Life-gated `A_BOLD` / `A_DIM`** (→ V3.11) — two thresholds: 0.6 and 0.2.

**`attr_t` OR accumulation** (→ V3.6) — build `attr_t attr = COLOR_PAIR(n); if (cond) attr |= A_BOLD;` before the draw call.

**Rocket always bold** — `COLOR_PAIR(r->color) | A_BOLD` hardcoded for the rising rocket. Always the brightest element while ascending.

**Independent particle colors** — `p->color = 1 + rand() % 7` at spawn. No color inheritance from parent rocket; explosions burst multicolored.

---

### particle_systems/brust.c
*Staggered explosion bursts with scorch persistence and flash cross-pattern.*

**Flash cross-pattern** (→ V5.11) — `'*'` center + `'+'` four cardinals, `COLOR_PAIR(C_YELLOW)|A_BOLD`, frame 0 only.

**Scorch persistence with `A_DIM`** (→ V5.10) — `scorch[]` array redrawn every frame before active particles.

**Life-gated brightness (single threshold)** (→ V3.11) — `life > 0.65 → A_BOLD`; scorch handles the dying-particle visual separately.

**`ASPECT=2.0f` in cell space** — `vx *= ASPECT` at particle spawn. Circular burst without full pixel-space physics — acceptable because burst spread is random anyway.

---

### particle_systems/kaboom.c
*Deterministic LCG explosions with pre-rendered Cell arrays and 3D blob z-depth coloring.*

**`Cell{ch, ColorID}` pre-render buffer** — `blast_render_frame()` fills `Cell cbuf[]` with no ncurses calls. `blast_draw()` blits it. Complete separation of geometry from I/O.

**6 blast theme structs** — `flash_chars[]` and `wave_chars[]` per theme. `ch = theme->wave_chars[ring_variant % len]`. Same geometry, different glyphs.

**3D blob z-depth → char + color** (→ V5.12) — far: `'.'`/`COL_BLOB_F`; mid: `'o'`/`COL_BLOB_M`; near: `'@'`/`COL_BLOB_N`.

**Role-named color IDs** (→ V3.12) — `COL_BLOB_F`, `COL_FLASH`, `COL_HUD` enum. HUD always registered yellow outside the theme block.

**LCG seed determinism** — `lcg_next = seed * 1664525 + 1013904223`. Same seed → same explosion every invocation.

---

### particle_systems/constellation.c
*Star constellation with stippled Bresenham lines, cell deduplication, proximity brightness.*

**`prev_px`/`prev_py` lerp** — `draw_px = prev_px + (px - prev_px) * alpha`. True interpolation (not extrapolation) — star velocities are non-linear.

**Distance-ratio attribute + stipple** (→ V5.4) — `ratio < 0.50 → A_BOLD, stipple=1`; `ratio < 0.75 → normal, stipple=1`; `ratio < 1.00 → normal, stipple=2`.

**`bool cell_used[rows][cols]` VLA** (→ V5.5) — stack-allocated per frame, zeroed with `memset`. First line to a cell claims it.

---

### flocking/flocking.c
*Boid flocking with 5 switchable modes, cosine palette cycling, proximity brightness.*

**Cosine palette → xterm-256 cube** (→ V3.8) — `init_pair()` called every N frames mid-loop with new cube index.

**`follower_brightness()` toroidal halo** (→ V5.6) — `ratio = toroidal_dist / PERCEPTION_RADIUS; return ratio < 0.35f ? A_BOLD : A_NORMAL`.

**Per-flock velocity char sets** — `k_boid_chars[3][8]`: each flock has its own 8 octant characters. Flocks remain visually distinct even when boids overlap.

```c
int octant = (int)((atan2f(-vy, vx) + M_PI) / (M_PI/4)) % 8;
char ch = k_boid_chars[flock_id][octant];
```

---

### fluid/sand.c
*Falling sand CA with dual-factor grain visuals and keyboard-controlled emitter.*

**`grain_visual(age, nb, &ch, &attr)`** (→ V5.13) — dual-factor: age → character density; neighbor count → A_BOLD.

**6 visual levels** — `` ` . o O 0 # `` from sparse (freshly fallen) to dense (compacted). Index advances with age.

**Source indicator `'|'`** — always drawn at emitter position with `CP_SOURCE|A_BOLD` regardless of whether grains are currently falling. No state check needed.

**Wind arrow glyph** — pick arrow from `"←↙↓↘→↗↑↖"` by wind direction; draw at HUD corner with `CP_WIND`. Immediate visual feedback without text.

**Fisher-Yates column shuffle** — column scan order randomized each tick. Removes left-to-right bias that otherwise piles sand asymmetrically.

---

### fluid/flowfield.c
*Perlin noise flow field with 4 visual themes, Unicode arrows, ring-buffer trails.*

**8 pairs, 4 runtime themes** — RAINBOW (8 hue octants), CYAN fade, GREEN fade, WHITE/grey. `color_apply_theme(t)` re-registers all 8 pairs. Same pair numbers; colors change.

**`angle_to_pair(angle)` — hue or age** — in RAINBOW: `pair = 1 + octant` (velocity direction → hue). In mono: `pair = 1 + trail_age_index` (time → brightness). Two completely different uses of the same 8 pairs, switched by theme.

**Unicode arrows via `addwstr`** (→ V4.9) — `const char *k_dirs[8]` = `{"→","↗","↑","↖","←","↙","↓","↘"}`.

**Ring-buffer trail coloring** (→ V5.14) — newest position gets pair 1 (brightest), oldest gets pair 8 (dimmest).

---

### raster/torus_raster.c
*UV torus rasterizer — establishes the shared raster pipeline pattern.*

**`Cell{ch, color_pair, bold}` + `cbuf[]`** (→ V2.5) — all raster math writes to the intermediate buffer; only `fb_blit()` touches ncurses.

**`zbuf[]` float depth buffer** — `FLT_MAX` each frame; write only when `frag_z < zbuf[idx]`.

**`luma_to_cell(luma, px, py)`** (→ V5.15) — Bayer dither → Bourke char → warm-to-cool pair, all in one call.

**7 warm-to-cool pairs** — pair 1 = red(196, bright/warm), pair 4 = green(neutral), pair 7 = magenta(201, dim/cool). High luma → warm; low luma → cool.

**Back-face cull always-on** — `dot(face_normal, view_dir) < 0 → skip`. Torus appears solid from outside.

---

### raster/cube_raster.c
*Unit cube rasterizer — same pipeline, adds toggleable cull and zoom.*

**Toggleable back-face cull `'c'`** — `cull_enabled` boolean flipped on keypress. Toggle lets you see inside the cube (cull off = inside-out rendering).

**Zoom via `'+'/'-'` adjusting FOV** — `fov` constant scales perspective matrix each frame. Interactive zoom with zero geometry changes.

**Same `cbuf`/`zbuf`/`fb_blit` pipeline** (→ V2.5) — demonstrates the pipeline handles different geometry with no changes to the ncurses output layer.

---

### raster/sphere_raster.c
*UV sphere rasterizer — same pipeline, UV lat/lon tessellation.*

**Same `cbuf`/`zbuf`/`fb_blit` pipeline** (→ V2.5) — same as torus and cube; only `tessellate_sphere()` differs.

**UV tessellation** — `x = sin(lat)*cos(lon)`, `y = cos(lat)`, `z = sin(lat)*sin(lon)`. Nested loops over `lat ∈ [0,π]`, `lon ∈ [0,2π]`. UV `(u,v) = (lon/2π, lat/π)`.

---

### raster/displace_raster.c
*UV sphere with real-time vertex displacement — the most complex raster file.*

**Vertex displacement modes via function pointer** — `float (*disp_fn)(Vec3 pos, float t, float amp, float freq)`. `'d'` key cycles `mode_idx`. Four modes: RIPPLE, WAVE, PULSE, SPIKY. Swap with no mesh or pipeline changes.

**Central-difference normal recomputation** — after displacing a vertex, normals are recomputed: `d_t = disp(pos+eps*T) - disp(pos-eps*T)`, then `T' = T*(2*eps) + N*d_t`, `N' = normalize(cross(T',B'))`. Without this, shading looks wrong on the deformed surface.

**Same `cbuf`/`zbuf`/`fb_blit` pipeline** (→ V2.5) — displacement is entirely in `tessellate_displace()`; `fb_blit()` unchanged.

---

### raymarcher/donut.c
*Parametric torus — no mesh, no SDF, no intermediate framebuffer.*

**No intermediate framebuffer** — each computed screen point calls `mvaddch` directly inside the render loop. Simplest possible 3D render path. Works because each parametric `(θ,φ)` point maps to a unique screen cell.

**8 grey-ramp pairs (235, 238, 241, 244, 247, 250, 253, 255)** (→ V3.5) — evenly spaced grey levels; `N·L` dot product maps to pair 1–8.

**8-color fallback** — when `COLORS < 256`: `pair_idx < 3 → A_DIM`; `pair_idx > 5 → A_BOLD`. Three apparent brightness tiers from one `COLOR_WHITE` pair (→ V3.7).

**Depth sort — painter's algorithm** (→ V5.16) — all points buffered, sorted by `z` descending, drawn back-to-front.

---

### raymarcher/wireframe.c
*Wireframe cube — Bresenham projected edges, slope characters, monochrome.*

**Bresenham projected edges** — project 8 cube vertices to screen; connect 12 edges with Bresenham. Character at each step selected by slope direction (→ V4.8).

**Arrow key rotation** — `KEY_UP/DOWN/LEFT/RIGHT` accumulate rotation angles each frame. `nodelay(TRUE)` (→ V6.1) means rotation advances only while a key is held.

**Monochrome — no `start_color()`** (→ V1.5) — all output uses terminal default. The entire visual is slope-character-based; color adds nothing.

---

### raymarcher/raymarcher.c
*Sphere-marching SDF raymarcher — sphere + plane, Blinn-Phong, soft shadow.*

**8 grey-ramp pairs + gamma correction** (→ V3.5) — `luma_gamma = powf(raw_luma, 1/2.2f)` before pair index lookup.

**Blinn-Phong → luma → grey pair** — `H = normalize(L + V)`; `specular = pow(max(dot(N,H),0), 32)`; `luma = kd*diffuse + ks*specular`; pair = `1 + (int)(luma * 7)`.

**`cbuf[]`/`fb_blit()` pattern** (→ V2.5) — unlike donut.c, the march loop writes to `cbuf`; `fb_blit()` is the sole ncurses boundary.

**Shadow ray** — secondary march from `p + N*eps` toward light. If it hits before the light, multiply diffuse by 0. Hard shadows at the cost of 2× SDF evaluations per lit pixel.

---

### raymarcher/raymarcher_cube.c
*SDF box raymarcher — adds finite-difference normals over raymarcher.c.*

**Finite-difference SDF normals** — `N = normalize( (sdf(p+ε,0,0)-sdf(p-ε,0,0), sdf(p+0,ε,0)-sdf(p-0,ε,0), sdf(p+0,0,ε)-sdf(p-0,0,ε)) )`. 6 extra SDF calls per hit; works for any SDF without analytic gradient derivation.

**Same grey-ramp + gamma + cbuf** (→ V3.5, V2.5) — identical output pipeline; only SDF function and normal method differ.

---

### raymarcher/raymarcher_primitives.c
*Multiple SDF primitives composited with min/max/smin, per-primitive material colors.*

**`min`/`max` SDF composition** — union = `min(a,b)`; intersection = `max(a,b)`; subtraction = `max(-a,b)`; smooth union = `smin(a,b,k)`.

**Per-primitive material color** — `map(p)` returns both distance and material ID. Material ID maps to a grey-ramp pair: sphere → light grey (pair 8), box → mid grey, torus → dark grey, etc.

**`cbuf[]` with per-material pair** (→ V2.5) — `cbuf[idx] = { ch, mat_id_to_pair[mat_id], bold }`. `fb_blit()` automatically uses the right pair per cell.

---

### artistic/bonsai.c
*Growing bonsai tree with recursive branch growth, transparent background, ACS borders.*

**`use_default_colors()` + `-1` background** (→ V1.3, V3.4) — branches float over the terminal's native background.

**ACS line-drawing chars for message box** (→ V4.5) — `ACS_ULCORNER`, `ACS_HLINE`, `ACS_VLINE`, etc. Portable box border on any terminal encoding.

**Branch boldness via conditional OR** — `attr_t a = COLOR_PAIR(cp) | (bold ? A_BOLD : 0)`. Older/thicker branches draw bold; young/thin ones at base brightness.

**Slope chars per branch direction** (→ V4.8) — `abs(dx) < abs(dy)/2 → '|'`; `abs(dy) < abs(dx)/2 → '-'`; `dx*dy > 0 → '\\'`; else `'/'`.

**Dual HUD rows** — `mvprintw(0, hx, ...)` for top bar; `mvprintw(rows-1, 0, ...)` for bottom bar. More status without crowding either edge.

**Message panel scrolling text** — `snprintf(buf, box_w-1, "%s", msg)` pre-clips to box width; `mvprintw(by+1, bx+1, "%s", buf)` inside ACS border.

---

### fractal_random/snowflake.c
*DLA crystal with D6 6-fold symmetry and distance-based coloring.*

**12-way simultaneous freeze** — when a walker freezes, all 12 D6 images (6 rotations × 2 reflections) are frozen atomically via `grid_freeze_symmetric()`. Each image is computed by rotating the displacement vector in Euclidean space and converting back to cell space.

**ASPECT_R=2 rotation correction** (→ V8.2 analogue for geometry) — `dy_e = dy / ASPECT_R` before rotation, `ry_e * ASPECT_R` after. Without this, the six rotated images are not 60° apart in physical pixels.

**Distance-based 6-color gradient** — `frac = dist / max_dist` maps to COL_ICE_1..6 (white tips → pale ice → bright cyan → medium teal → ocean blue → light blue core). `max_dist` updates as the crystal grows, stretching the gradient.

**Context-sensitive draw characters** — `grid_char_at()` scans the 8 neighbours of each frozen cell and picks `*` `|` `-` `+` `/` `\` based on which neighbors are occupied. Isolated cells get `*`; horizontal runs get `-`; vertical runs get `|`; junctions get `+`.

**Walker visibility** — active walkers drawn as xterm 75 (sky blue) `·` with no A_DIM, ensuring the random walk is visible against the black background.

---

### fractal_random/coral.c
*Anisotropic DLA with 8 bottom seeds and directional sticking.*

**Bottom-seeded 8-seed init** — 8 evenly spaced cells along the bottom row are frozen at startup, giving the growth multiple independent nucleation points that merge into a connected coral structure.

**Downward walker drift** — walkers spawn at the top and move with 50% downward bias. Combined with directional sticking (above=0.90, side=0.40, below=0.10), branches grow upward like coral polyps.

**Height-based color banding** — frozen cells are colored by their row: bottom rows get COL_CORAL_1 (coral red), upper rows cycle through violet, yellow, lime, teal, lemon. Six vivid pairs, all 256-color with 8-color fallback.

**Auto-reset on max height** — `scene_tick()` checks if the tallest frozen branch has reached `rows / 4` and resets the scene, keeping the animation cycling.

---

### fractal_random/sierpinski.c
*Sierpinski triangle via chaos game (IFS).*

**Three-vertex IFS** — `ifs_step()` picks one of three vertices uniformly at random and moves the current point halfway toward it. After 20 warm-up iterations (discarded), all subsequent points lie on the attractor.

**Vertex color tracking** — the color of each plotted point is `which + 1` where `which` is the last chosen vertex index. This partitions the triangle into three colored sub-triangles (cyan/yellow/magenta), with the same tricolor pattern repeating at every scale.

**N_PER_TICK=500, TOTAL_ITERS=50000** — builds the triangle gradually over ~100 ticks. `DONE_PAUSE_TICKS=90` holds the completed triangle for ~3 s before resetting.

**Equilateral scaling** — `scale_x = scale_y × ASPECT_R` ensures the triangle looks equilateral on non-square terminal cells. `x_off` centers it horizontally.

---

### fractal_random/fern.c
*Barnsley Fern via 4-transform IFS.*

**4-transform weighted IFS** — each `ifs_step()` picks a transform by cumulative probability (1%, 86%, 93%, 100% cutpoints). The `(a,b,c,d,e,f)` matrix plus probability is stored in a `static const IFSTransform` array.

**Independent x/y scale** — fern y/x aspect ≈ 4:1 in math space. Separate `scale_y` and `scale_x` (cols × 0.45 / x_range) map to a comfortable terminal width without squashing.

**N_PER_TICK=400, TOTAL_ITERS=80000** — denser point cloud than sierpinski for a fuller-looking fern. 20 warm-up iterations before plotting begin.

**Green gradient 5 pairs** — points are colored by which transform produced them: stem=40 (medium green), main=76 (bright green), left=118 (lime), right=154 (yellow-lime), tips=190 (lemon).

---

### fractal_random/julia.c
*Julia set with Fisher-Yates random pixel reveal and 6 preset cycling.*

**Fisher-Yates shuffled order** (→ V5 new technique) — `grid_shuffle()` fills `order[0..n-1]` then does a Fisher-Yates shuffle. `pixel_idx` advances PIXELS_PER_TICK per tick, revealing pixels in random order. Image materialises from scattered noise.

**6 presets cycle automatically** — on reset, `preset_idx` advances. Presets vary `cr`, `ci` (the fixed complex constant c). HUD shows preset name with `mvprintw`.

**Fire palette** — COL_INSIDE=231 (white), COL_C2=226 (yellow), COL_C3=208 (orange), COL_C4=196 (red), COL_C5=124 (dark red). Slow-escape pixels (near boundary) are bright; fast-escape pixels are dim outer halos.

**MAX_ITER=128, PIXELS_PER_TICK=60** — complete reveal in ~40 s per preset. `im_half=1.3f`; `re_half = im_half × cols/rows / ASPECT_R` for correct aspect.

---

### fractal_random/mandelbrot.c
*Mandelbrot set with zoom presets and electric neon palette.*

**re_center / im_center in Grid** — unlike julia.c (always centred at origin), the mandelbrot grid carries `re_center`, `im_center`, `zoom` to map each pixel to the correct complex coordinate for each preset.

**6 zoom presets** — from full set (zoom=1.3) through seahorse valley (0.15), deep spiral (0.02), mini mandelbrot (0.08). Each preset selects a visually interesting region of the Mandelbrot set.

**Electric neon palette** — COL_INSIDE=201 (magenta), COL_C2=226 (yellow), COL_C3=82 (lime), COL_C4=51 (cyan), COL_C5=141 (purple). Contrasts with julia.c's fire scheme; makes the two files visually distinct.

**MAX_ITER=256** — higher than julia.c for finer boundary detail, especially in deep zoom presets.

---

### fractal_random/koch.c
*Koch snowflake — recursive midpoint subdivision with Bresenham rasterization.*

**Recursive segment buffer** — `subdivide(segs, n)` uses static globals `g_seg_buf[]`/`g_seg_n`. Level n has 3×4ⁿ segments; level 5 has 3072. MAX_SEGS=4096 covers all levels.

**Adaptive segs_per_tick** — `segs_per_tick = n_segs / 60 + 1` gives approximately 2 s per level regardless of segment count. Level 1 draws all 12 segments in one tick; level 5 draws ~51 per tick.

**Bresenham rasterization** — `rasterize_seg()` uses integer Bresenham to paint single-cell-wide lines. Each segment receives a color from a 5-color gradient (cyan→teal→lime→yellow→white) indexed by draw order.

**Level cycling 1→5** — on completing each level, `HOLD_TICKS=45` ticks pause before clearing and moving to the next level. After level 5, wraps back to 1.

**Keys: r=restart, n=next level** — in addition to standard q/p/[/].

---

### fractal_random/lightning.c
*Fractal binary-tree lightning with glow halo and depth coloring.*

**Tip struct with lean bias** — each active tip carries `row`, `col`, `lean` (−2..2), `steps_since_fork`. `lean/2` (rounded) gives the column delta per step. Lean persists, so branches travel diagonally at a consistent angle.

**Fork probability after MIN_FORK_STEPS** — after a minimum step count, each tip independently tests `rand() % 100 < FORK_PROB`. On fork, two child tips inherit `lean ± 1`. This produces the spreading binary-tree structure.

**State machine ST_GROWING → ST_STRIKING → ST_FADING** — ensures the bolt grows fully before fading, giving a brief "flash" hold at full extension.

**Glow halo at draw time** — `draw_glow()` iterates Manhattan neighbors of radius 1 and 2 around each frozen cell; empty neighbors get dim overlay characters (`|`, `.`). Drawn before the bolt itself so the bolt overwrites the halo at bolt positions.

**Depth color** — `row < rows/3` = xterm 45 (light blue), `row < 2*rows/3` = xterm 51 (teal), else xterm 231 (white). Bolt reads as energetic at the top, incandescent at ground strike.

---

### fractal_random/buddhabrot.c
*Buddhabrot density accumulator — orbital trajectory heatmap, purple→white nebula palette.*

**Two-pass orbit sampling** — Pass 1 tests whether c escapes within max_iter steps. Pass 2 (only for qualifying samples) re-iterates and increments `counts[row][col]` for each orbit point inside the display region. No orbit buffer is allocated; the two-pass design re-derives the orbit cheaply.

**Anti-Buddhabrot mode** — same algorithm but traces orbits that do NOT escape (bounded/interior orbits). Reveals the interior structure of the Mandelbrot set as a dense attractor pattern.

**Log normalization with mode-aware floor** — `t = log(1+count)/log(1+max_count)`. Anti mode uses floor=0.25 to suppress the scattered dots caused by transient cells with count=1 against a max_count of millions. Buddha mode uses floor=0.05 to preserve low-density orbital detail:
```c
float floor = anti ? 0.25f : 0.05f;
if (t < floor) return 0;   /* invisible */
```

**Five presets cycling automatically** — buddha-500, buddha-2000, anti-100, anti-500, anti-1000. After TOTAL_SAMPLES=150000 the image pauses then advances. `n` / `r` skips to next preset immediately.

**Nebula color palette** — five levels: dark blue-purple (55) → violet (93) → light purple (141) → lavender-pink (183) → white bold (231). Characters: `.` `:` `+` `#` `@`.

---

### Artistic/bat.c
*Three groups of ASCII bats in Pascal-triangle formation flying outward from centre.*

**Pascal-triangle formation** — row r has r+1 bats; total = (n_rows+1)*(n_rows+2)/2. Flat index k maps to row/position via triangular-number inverse (`bat_form_offset`). Leader sits at apex; each successive row spreads wider by SPREAD_PX per slot.

**`+` / `-` live resize** — changes n_rows (1–6) while groups are in flight. New bats are placed at the correct formation position relative to the current leader without interrupting motion. Maximum 28 bats per group (n_rows=6).

**Wing animation** — four-frame cycle: `/`, `-`, `\`, `-` for the left wing; mirrored for the right. All bats in a group share the same phase tick, giving synchronized flapping.

**Three groups, staggered launch** — groups launch 30 ticks apart (STAGGER_TICKS). Colors: group 0 = xterm 141 (light purple), group 1 = xterm 87 (electric cyan), group 2 = xterm 213 (pink-magenta). Six preset angles (330°/210°/90°/45°/135°/270°) distribute flight directions each cycle.

**Keys: `+`/`-` bat rows, `r`/`n` reset/next** — in addition to standard `q`/`p`/`[`/`]`.

---

### fluid/wave.c
*FDTD 2-D wave equation with five togglable point sources.*

**Triple-buffer FDTD** — `u_prev`, `u_cur`, `u_new` planes rotate each step. Explicit scheme: `u_new = 2·u_cur − u_prev + c²·∇²u_cur`. CFL stability: `c = 0.45` (max safe ≈ 0.707). Per-step damping factor dissipates energy.

**Signed amplitude → 9-level ramp** — Negative (troughs) map to cool/dim colors; near-zero maps to blank; positive (crests) map to warm/bright colors. The sign distinction makes interference fringes immediately readable.

**Five sources, offset frequencies** — Each source oscillates at a slightly different frequency. Beating between adjacent pairs creates slowly shifting interference patterns. Keys `1`–`5` toggle each source independently.

**Dynamic theme** — 4 themes, re-register color pairs on `t` keypress (→ V3.10).

---

### fluid/reaction_diffusion.c
*Gray-Scott two-chemical reaction-diffusion: spots, stripes, coral, worms.*

**9-point isotropic Laplacian** — `∇²u ≈ 0.20·(N+S+E+W) + 0.05·(NE+NW+SE+SW) − u`. More rotationally symmetric than the 4-point stencil, producing rounder spots and stripes.

**Dual-grid ping-pong** — `grid[0]` and `grid[1]` alternate as read/write each step. Active index flips between 0 and 1. This ensures all cells read the same generation (simultaneous update, not sequential).

**Color by V concentration** — The V channel (catalyst) is mapped through a theme palette. High V = pattern; low V = background. Multiple themes swap the palette live.

**600-step warm-up** — Pre-computed before first display so patterns are already developed at startup, not just seeds.

---

### physics/double_pendulum.c
*Chaotic double pendulum with ghost trajectory and fading trail.*

**RK4 integration** — 4th-order Runge-Kutta for the Lagrangian equations of motion. Lower-order integrators (Euler, Verlet) accumulate phase errors on the Lyapunov time-scale that look identical to real chaos — RK4 keeps the simulation honest for longer.

**Ghost trajectory** — A second pendulum starts with `θ₁ + GHOST_EPSILON` offset. Both are identical at first; after ~3–5 s they diverge completely, demonstrating sensitive dependence on initial conditions. The HUD shows the angular separation.

**Ring-buffer trail with warm-to-cool fade** — Recent trail positions render bright red/orange; older positions fade to dim grey. Same ring-buffer pattern as flowfield.c (→ V5.14), but with a warm-to-cool color gradient instead of age-based alpha.

**Subpixel pivot** — The pivot point is at screen center in pixel space; bob positions are converted to terminal cells at draw time.

---

### artistic/epicycles.c
*DFT of parametric shapes animated as a chain of rotating arms.*

**DFT arm chain** — Shape sampled into N=256 complex points, DFT computed, coefficients sorted by amplitude (largest first). At angle φ, each arm contributes `(|Z[n]|/N)·exp(i·(freq_n·φ + arg Z[n]))`. Arms are chained from the pivot; the tip traces the original shape.

**Auto-add to show convergence** — Every `AUTO_ADD_FRAMES = 12` frames one more epicycle is added. Starting with 1 arm and slowly growing to all 256 shows how Fourier series builds up a shape from pure sinusoids.

**Subpixel coordinates** — `CELL_W = 8` sub-pixel space (same as flocking.c → §4 coords). Arm positions stored in pixel space, divided by `CELL_W`/`CELL_H` at draw time for isotropic proportions.

**Orbit circles** — The `N_CIRCLES` largest arms draw their orbit as `·` dots when toggled with `c`. Clarifies why specific shapes emerge from the harmonic chain.

---

### artistic/leaf_fall.c
*Procedural ASCII tree followed by matrix-rain leaf fall.*

**ST_DISPLAY / ST_FALLING / ST_RESET state machine** — Three states with distinct tick rates. `spc` skips the current state. Each new tree uses fresh random parameters so no two trees are identical.

**Recursive branching** — Trunk drawn bottom-up; branches grown from a stack (`BSTACK`) with heading angle, length, depth. At each node the tree probabilistically splits into two sub-branches with randomised spread. Leaves placed at terminal tips.

**Matrix-rain leaf fall** — Each leaf column has a white `*` head moving downward at `FALL_NS` rate with a 7-character green trail behind it. Same two-pass structure as matrix_rain.c (→ V5.3). Columns stagger their start by a random delay (`MAX_START_DELAY`) for organic appearance.

---

### geometry/string_art.c
*Mathematical string art: N nails on a circle, threads connect i → round(i·k) mod N.*

**k-multiplier thread art** — Multiplier `k` drifts continuously. At integer k values cardioid (k=2), nephroid (k=3), deltoid (k=4), astroid (k=5) emerge. Speed modulates near integers (`mult = 0.15 + 1.70·(dist²·4)`) so named shapes hold long enough to read.

**Slope-based thread characters** — Each thread line uses one of `-`, `|`, `/`, `\` chosen by visual slope with aspect correction: `vs = |dy·2/dx|`; `vs < 0.577` → `-`; `vs > 1.732` → `|`; else sign-correct `/` or `\`. Makes individual threads look like actual lines rather than scatter.

**Circle rim + nail markers** — `RIM_STEPS = 2000` points draw the circle as `·`. Nails drawn as bold `o` on top. Provides clear visual landmarks for the geometry.

**12-step rainbow** — Fixed 256-color cycle covering spectrum. Thread `i` uses `CP_T0 + (i % 12)`, giving each nail its own hue.

---

### artistic/cellular_automata_1d.c
*Wolfram elementary rules animated as a build-down pattern.*

**3-neighbor bitmask rule** — `next[c] = (rule >> ((l<<2)|(m<<1)|r)) & 1`. The 8-bit rule number directly encodes the 8 possible 3-neighbor combinations. Any of the 256 rules can be set live by typing a number.

**Build-down animation** — Pattern builds from row 0 downward; `g_gen` tracks how many rows are visible. `ST_BUILD` adds one row per `g_delay` ticks; `ST_PAUSE` (90 ticks, ~3 s) holds the complete pattern before resetting.

**A_REVERSE title bar in class color** — Row 0 shows rule name and number in `attron(COLOR_PAIR(cp) | A_BOLD | A_REVERSE)`. The background block in the class color immediately identifies whether the rule is fixed/periodic/chaotic/complex/fractal without obscuring the CA grid.

**Live digit input** — Typing 1–3 digits accumulates a rule number, applying immediately after 3 digits or Enter. Lets the user explore any Wolfram rule without cycling through presets.

---

### artistic/life.c
*Conway's Game of Life and five rule variants with population histogram.*

**B/S bitmask rules** — `uint16_t birth` and `uint16_t survive` where bit N is set if neighbor count N triggers the event. Testing: `(mask >> n) & 1`. Adding a new rule requires only two bitmask literals; the step function is rule-agnostic.

**Double-buffered grid swap** — `g_grid[2][MAX_ROWS][MAX_COLS]`. After computing the next generation into buffer `1 - g_buf`, `g_buf` flips. The step function always reads from `g_buf` and writes to `1 - g_buf`.

**Dynamic color per rule** — `color_apply()` re-registers `CP_LIVE` with the current rule's 256-color index when the rule changes (→ V3.13 dynamic color re-registration). Each of the 6 rules gets a distinct hue.

**Population histogram** — `HIST_LEN = 512` ring buffer stores per-generation counts. Bottom 3 rows render as a bar chart growing upward: `level = pop/max_pop · HIST_ROWS · 2`. Shows oscillations, extinction events, and the transition to stable configurations.

---

### artistic/langton.c
*Langton's Ant and multi-colour turmites: rule-string-driven ant automata.*

**Rule string as cell-state turn sequence** — `rule[s % g_n_colors]` gives the turn ('R' or 'L') for a cell in state `s`. After turning, the cell advances: `(state + 1) % g_n_colors`. "RL" is the classic 2-state Langton's ant; "LLRR", "LRRL" etc. produce more complex structures.

**Modulo-4 direction cycling** — Four directions (N/E/S/W). Turn right: `(dir + 1) % 4`. Turn left: `(dir + 3) % 4` (equivalent to `(dir - 1 + 4) % 4`). Toroidal wrap on move: `(r + DR[dir] + g_rows) % g_rows`.

**Multi-ant shared grid** — 1–3 ants share the same `g_grid`. They interact because each reads and modifies the same cell states. Produces emergent structures not seen with a single ant.

**High step rate** — `STEPS_DEF = 200` ant steps per frame; `+`/`-` doubles/halves up to `STEPS_MAX = 2000`. Needed because the classic "RL" highway requires ~10,000 steps to emerge.

---

### artistic/cymatics.c
*Chladni nodal-line figures with morph animation between 20 modes.*

**Chladni formula** — `Z(x,y) = cos(m·π·x)·cos(n·π·y) − cos(n·π·x)·cos(m·π·y)` where `(x,y) ∈ [0,1]`. Nodal lines where `Z ≈ 0` are where sand collects on a vibrating plate. 20 mode pairs `(m,n)` with `1 ≤ m < n ≤ 7`.

**Blended morph** — `z = (1−t)·z1 + t·z2` where `t` advances `MORPH_SPEED = 0.025f` per tick. Smooth transition between modes takes ~1.3 s. After morphing, `ST_HOLD` pauses for 4 s (120 ticks) before the next morph.

**Nodal glow bands** — Five threshold bands around the nodal line use progressively fainter characters (`@#*+.`). Cells beyond `|Z| ≥ 0.40` are blank. The innermost band (`|Z| < 0.04`) is white `CP_NODE`; outer bands are colored by sign (CP_POS vs CP_NEG).

**Four themes** — Classic/Ocean/Ember/Neon. Each theme has different `CP_POS` and `CP_NEG` color pairs. `color_init()` re-registers all pairs on theme change (→ V3.10).

---

### artistic/wator.c
*Wa-Tor predator-prey ocean with fish/shark population oscillations.*

**`g_moved[][]` double-process guard** — When a shark moves to a cell (or eats a fish there), `g_moved[r][c] = 1`. Later in the same tick, the entity at that cell is skipped if `g_moved` is set. Without this, a shark moved by the shuffled order could act twice — once at origin, once at destination — per tick.

**Fisher-Yates shuffle** — All `rows × cols` cell indices are shuffled before processing each tick. Unshuffled top-left processing order would make fish drift right/down preferentially. The shuffle is O(n) at tick start; the skip of EMPTY cells is O(1) per index.

**Dual-panel histogram** — 4-row bottom strip: upper 2 rows = fish (cyan, scale=max_pop/2), lower 2 rows = sharks (red, scale=max_pop/10). Fish scale is 5× coarser than sharks because fish vastly outnumber sharks at stable equilibrium.

---

### fractal_random/sandpile.c
*Bak-Tang-Wiesenfeld Abelian Sandpile with self-organised criticality.*

**BFS avalanche with `g_inq[]` dedup** — Unstable cells enqueued via `enq(r,c)` which guards with `if (g_inq[r][c]) return`. The circular queue head/tail allows re-enqueueing of the same cell in a later wave without overflow. `QMAX = MAX_ROWS × MAX_COLS + 1` — one extra slot for the circular boundary condition.

**Vis mode** — `avalanche_step(full=0)` breaks after one topple. The caller renders between calls, showing the cascade propagate cell by cell in bright red. `avalanche_step(full=1)` drains to empty — the full cascade completes before the next frame.

**Curvature-like coloring** — Grain count 0–3 maps to `' ' '.' '+' '#'` with colors dim-blue / green / bright-gold. The visual gradient encodes how close each cell is to instability — the near-critical ring of `#` cells surrounding the centre is the self-organised critical state made visible.

---

### raymarcher/metaballs.c
*6 SDF metaballs on Lissajous orbits, smooth-min blending, curvature coloring.*

**`smin(a,b,k)` polynomial blend** — `h = max(k − |a−b|, 0) / k`. Blend term `h²·k/4` extends the isosurface beyond either sphere when `|a−b| < k`. Small `k` ≈ hard min (separate balls); large `k` ≈ merged blob. Adjust live with `j`/`k` keys.

**Tetrahedron normal** — 4 SDF evaluations at `(±e,±e,±e)` vertices recover the gradient: `nx = f0−f1−f2+f3`, `ny = −f0+f1−f2+f3`, `nz = −f0−f1+f2+f3`. Saves 33% over 6-point central differences with no accuracy loss.

**Curvature coloring** — 7-point Laplacian stencil (6 neighbours + centre, `eps=CURV_EPS=0.06`): high curvature (sphere peaks) → high band → warm hue; low curvature (merged saddle) → low band → cool hue. 4 themes × 8 bands = 32 `init_pair` calls at startup; theme switch requires no re-registration.

**2×2 block canvas** — `CELL_W=2, CELL_H=2` renders each canvas pixel as a 2×2 terminal block. Aspect-corrects via `phys_aspect = (ch·CELL_H·CELL_ASPECT)/(cw·CELL_W)`. Reduces per-frame pixel count 4× for a more comfortable frame rate despite the heavy SDF math.

---

### geometry/lissajous.c
*Harmonograph: two damped oscillators, phase drifts, morphing Lissajous figures.*

**Age-based rendering** — iterates `i = N_CURVE_PTS-1 → 0` (oldest inner first, newest outer last). `age = i/(N-1)`: 0=newest → level 0 (brightest `#`, `A_BOLD`), 1=oldest → level 3 (dimmest `.`). Newest overwrites shared cells so the bright outer ring always wins.

**Phase dwell** — `key_period = π / max(fx, fy)`. Near each symmetric phase (where the figure closes on itself), drift is multiplied by `DWELL_SPEED=0.25` and linearly ramps back to 1× within `DWELL_WIDTH=0.25` of the period. Each named figure dwells visibly before transitioning.

**T_MAX / DECAY normalization** — `T_MAX = N_LOOPS·2π/min(fx,fy)` always gives 4 complete cycles of the slower oscillator. `DECAY = DECAY_TOTAL/T_MAX` so amplitude reaches ~1% at T_MAX for every ratio — consistent spiral depth regardless of frequency pair.

**4 themes × 4 levels** — 16 `init_pair` calls + 1 HUD pair. Theme cycling is a single integer increment; no re-registration needed.

---

## Quick-Reference Matrix

`✓` = technique present. `—` = not used.

| File | erase() | diff-clear | cbuf+zbuf | grey-ramp | Bayer | F-Stein | theme-swap | use_default | ACS | Unicode |
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
| kaboom | ✓ | — | ✓* | — | — | — | ✓ | — | — | — |
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
| snowflake | ✓ | — | — | — | — | — | — | — | — | — |
| coral | ✓ | — | — | — | — | — | — | — | — | — |
| sierpinski | ✓ | — | — | — | — | — | — | — | — | — |
| fern | ✓ | — | — | — | — | — | — | — | — | — |
| julia | ✓ | — | — | — | — | — | — | — | — | — |
| mandelbrot | ✓ | — | — | — | — | — | — | — | — | — |
| koch | ✓ | — | — | — | — | — | — | — | — | — |
| lightning | ✓ | — | — | — | — | — | — | — | — | — |
| buddhabrot | ✓ | — | — | — | — | — | — | — | — | — |
| bat | ✓ | — | — | — | — | — | — | — | — | — |
| wave | ✓ | — | — | — | — | — | ✓ | ✓ | — | — |
| reaction_diffusion | ✓ | — | — | — | — | — | ✓ | ✓ | — | — |
| double_pendulum | ✓ | — | — | — | — | — | — | ✓ | — | — |
| epicycles | ✓ | — | — | — | — | — | — | ✓ | — | — |
| leaf_fall | ✓ | — | — | — | — | — | — | ✓ | — | — |
| string_art | ✓ | — | — | — | — | — | — | ✓ | — | — |
| cellular_automata_1d | ✓ | — | — | — | — | — | — | ✓ | — | — |
| life | ✓ | — | — | — | — | — | ✓ | ✓ | — | — |
| langton | ✓ | — | — | — | — | — | — | — | — | — |
| cymatics | ✓ | — | — | — | — | — | ✓ | ✓ | — | — |
| wator | ✓ | — | — | — | — | — | — | — | — | — |
| sandpile | ✓ | — | — | — | — | — | — | — | — | — |
| metaballs | ✓ | — | ✓ | — | — | — | ✓ | ✓ | — | — |
| lissajous | ✓ | — | — | — | — | — | ✓ | ✓ | — | — |

\* kaboom uses a `Cell[]` pre-render buffer (→ V2.5 pattern) not a `zbuf[]`.

**Column key:**
- `erase()` — standard per-frame full erase (V2.2)
- `diff-clear` — selective cell erase, no full `erase()` (V2.6)
- `cbuf+zbuf` — intermediate framebuffer + depth buffer, `fb_blit()` as sole ncurses boundary (V2.5)
- `grey-ramp` — xterm-256 grey pairs 235–255 (V3.5)
- `Bayer` — 4×4 ordered dithering (V5.7)
- `F-Stein` — Floyd-Steinberg error diffusion (V5.8)
- `theme-swap` — runtime `init_pair()` re-registration (V3.10)
- `use_default` — `use_default_colors()` for transparent background (V1.3)
- `ACS` — ACS line-drawing characters (V4.5)
- `Unicode` — `addwstr` for multi-byte glyphs (V4.9)

---

## Technique Index

| Technique | V-section | Files |
|---|---|---|
| Standard init sequence | V1.1 | all |
| `endwin()` + `atexit` cleanup | V1.2 | all |
| `use_default_colors()` transparent bg | V1.3 / V3.4 | matrix_rain, bonsai |
| `cbreak()` vs `raw()` | V1.4 | all (cbreak); aspect_ratio (raw) |
| Monochrome — no `start_color()` | V1.5 | wireframe |
| `curscr`/`newscr` double buffer | V2.1 | all |
| `erase()` vs `clear()` | V2.2 | all |
| `wnoutrefresh` + `doupdate` | V2.3 | all |
| Avoid manual front/back windows | V2.4 | all (avoid); aspect_ratio (example) |
| `cbuf[]` + `fb_blit()` framebuffer | V2.5 | torus/cube/sphere/displace_raster, raymarcher* |
| Diff-based selective clearing | V2.6 | aafire_port |
| Color pairs — `init_pair`/`COLOR_PAIR` | V3.1 | all |
| 256-color vs 8-color fallback | V3.2 | all |
| xterm-256 palette index reference | V3.3 | all color files |
| `-1` background for transparency | V3.4 | matrix_rain, bonsai |
| Grey ramp 235–255 for luminance | V3.5 | donut, raymarcher* |
| `attr_t` OR accumulation | V3.6 | all |
| `A_BOLD`/`A_DIM` brightness tiers | V3.7 | all |
| Cosine palette cycling | V3.8 | flocking |
| Shade enum → composite `attr_t` | V3.9 | matrix_rain |
| Encapsulated theme struct | V3.10 | fire, aafire_port |
| Life-gated `A_BOLD`/`A_DIM` | V3.11 | fireworks, brust |
| Role-named color IDs | V3.12 | kaboom |
| `mvwaddch` core write call | V4.1 | all |
| `(chtype)(unsigned char)` double cast | V4.2 | all |
| `wattron`/`wattroff` bracket | V4.3 | all |
| `mvprintw` HUD text | V4.4 | all |
| ACS line-drawing chars | V4.5 | bonsai |
| Paul Bourke 92-char ramp | V4.6 | rasters, raymarchers, fire |
| Directional velocity → glyph | V4.7 | flowfield, flocking |
| Slope chars `/\|-` | V4.8 | spring_pendulum, bonsai, wireframe |
| Unicode glyphs via `addwstr` | V4.9 | flowfield |
| Draw order — last write wins | V5.1 | all |
| HUD always on top | V5.2 | all |
| Two-pass rendering | V5.3 | matrix_rain |
| Stippled Bresenham distance fade | V5.4 | constellation |
| `cell_used[][]` line deduplication | V5.5 | constellation |
| Proximity brightness `A_BOLD` | V5.6 | flocking |
| Bayer 4×4 ordered dithering | V5.7 | rasters |
| Floyd-Steinberg error diffusion | V5.8 | fire, aafire_port |
| Luminance → warm-to-cool color | V5.9 | rasters |
| Scorch mark accumulation | V5.10 | brust |
| Flash cross-pattern | V5.11 | brust |
| Z-depth → char + color | V5.12 | kaboom |
| Dual-factor visual function | V5.13 | sand |
| Ring-buffer trail coloring | V5.14 | flowfield |
| `luma_to_cell` dither+ramp+color | V5.15 | rasters |
| Depth sort — painter's algorithm | V5.16 | donut |
| `nodelay` non-blocking input | V6.1 | all |
| `keypad` arrow/function keys | V6.2 | all |
| `typeahead(-1)` atomic write | V6.3 | all |
| `noecho` suppress key echo | V6.4 | all |
| `curs_set(0)` hide cursor | V6.5 | all |
| `SIGWINCH` resize signal | V7.1 | all |
| `volatile sig_atomic_t` flags | V7.2 | all |
| `endwin→refresh→getmaxyx` sequence | V7.3 | all |
| `atexit(cleanup)` guaranteed restore | V7.4 | all |
| DLA walker + freeze | — (see Architecture §22) | snowflake, coral |
| D6 12-way symmetric freeze | — (see Architecture §22) | snowflake |
| Distance-based color gradient | — (see COLOR §18) | snowflake |
| Context-sensitive draw chars | — (see Architecture §22) | snowflake |
| Anisotropic sticking probability | — (see Architecture §22) | coral |
| IFS chaos game | — (see Architecture §22) | sierpinski, fern |
| IFS vertex color tracking | — (see COLOR §20) | sierpinski, fern |
| Escape-time band coloring | — (see COLOR §19) | julia, mandelbrot |
| Fisher-Yates random pixel reveal | — (see Architecture §22) | julia, mandelbrot |
| Koch midpoint subdivision | — (see Architecture §22) | koch |
| Adaptive segs_per_tick | — (see Architecture §22) | koch |
| Bresenham segment rasterization | — (see Architecture §22) | koch |
| Recursive tip branching | — (see Architecture §22) | lightning |
| Depth-position coloring | — (see COLOR §21) | lightning, koch |
| Glow halo at draw time | — (see Architecture §22) | lightning |
| Two-pass orbit sampling | — (see Architecture §22) | buddhabrot |
| Log density normalization | — (see COLOR §22) | buddhabrot |
| Mode-aware invisible floor | — (see COLOR §22) | buddhabrot |
| Pascal-triangle formation index | — (see Architecture §22) | bat |
| Formation world-space rotation | — (see Architecture §22) | bat |
| Synchronized wing animation | — | bat |
| FDTD triple-buffer signed ramp | — (see Architecture §23) | wave |
| Gray-Scott dual-grid ping-pong | — (see Architecture §24) | reaction_diffusion |
| 9-point isotropic Laplacian | — (see Architecture §24) | reaction_diffusion |
| RK4 integration + ghost trajectory | — (see Architecture §25) | double_pendulum |
| DFT epicycle arm chain | — (see Architecture §26) | epicycles |
| Build-down CA animation | — (see Architecture §29) | cellular_automata_1d |
| B/S bitmask cellular automaton | — (see Architecture §30) | life |
| Dynamic color per rule switch | V3.13 | life, wave, reaction_diffusion, cymatics |
| Population histogram ring buffer | — (see Architecture §30) | life |
| Rule-string turmite ant | — (see Architecture §31) | langton |
| Chladni nodal-glow band rendering | — (see Architecture §32) | cymatics |
| ST_HOLD / ST_MORPH blended morph | — (see Architecture §32) | cymatics |
| Fisher-Yates shuffle + g_moved guard | — (see Architecture §33) | wator |
| Dual fish/shark histogram panels | — (see Architecture §33) | wator |
| BFS queue + g_inq dedup | — (see Architecture §34) | sandpile |
| Instant vs vis avalanche mode | — (see Architecture §34) | sandpile |
| Polynomial smooth-min SDF blend | — (see Architecture §35) | metaballs |
| Tetrahedron 4-point normal | — (see Architecture §35) | metaballs |
| Laplacian curvature coloring | — (see Architecture §35) | metaballs |
| Soft shadow penumbra (sk·h/t) | — (see Architecture §35) | metaballs |
| 2×2 block canvas downsampling | — (see Architecture §35) | metaballs |
| Damped oscillator parametric curve | — (see Architecture §36) | lissajous |
| Phase dwell (symmetric Lissajous) | — (see Architecture §36) | lissajous |
| Age-based brightness decay rendering | — (see Architecture §36) | lissajous |
| T_MAX / DECAY ratio normalization | — (see Architecture §36) | lissajous |
| k-multiplier thread art | — (see Architecture §28) | string_art |
| Slope chars via aspect-corrected gradient | V4.8 | string_art, spring_pendulum, bonsai |
| A_REVERSE title bar in class color | — | cellular_automata_1d |
| Matrix-rain leaf fall with stagger | V5.3 | leaf_fall |
| Sleep before render | V8.1 | all |
| `getmaxyx` vs `LINES`/`COLS` | V8.2 | all |
| `CLOCK_MONOTONIC` no NTP jumps | V8.3 | all |
| Common ncurses bugs reference | V8.4 | — |
| De Bruijn pentagrid O(1) tile lookup | — (see Architecture §39) | penrose |
| Pentagrid edge detection + directional chars | — (see Architecture §39) | penrose |
| Diamond-square heightmap generation | — (see Architecture §40) | terrain |
| Thermal weathering erosion | — (see Architecture §40) | terrain |
| Bilinear interp grid → terminal | — (see Architecture §40) | terrain |
| Sinusoidal aurora curtains + envelope | — (see Architecture §38) | aurora |
| Deterministic star hash (no storage) | — (see Architecture §38) | aurora |
| Demoscene plasma sin-sum + palette cycle | — (see Architecture §38) | plasma |
| Hypotrochoid parametric curve | — (see Architecture §38) | spirograph |
| Float canvas exponential decay trail | — (see Architecture §38) | spirograph |
| Langevin Brownian seed motion | — (see Architecture §38) | voronoi |
| d2−d1 Voronoi edge detection | — (see Architecture §38) | voronoi |
| Explicit spring forces + symplectic Euler | — (see Architecture §37, Master §R1) | cloth |
| Jakobsen constraint failure with pins | — (see Master §R1) | cloth |
| Softened N-body gravity Verlet | — (see Architecture §37, Master §R2) | nbody |
| Lorenz RK4 + ghost trajectory chaos demo | — (see Architecture §37, Master §R3) | lorenz |
| Rotating orthographic 3-D projection | — (see Architecture §37) | lorenz |
| Phase precomputation k·r → g_phase[][] | — (see Master §T1) | wave_interference |
| Signed 8-level color ramp CP_N3..CP_P3 | COLOR §23 | wave_interference |
| 7-segment bitmask + particle ownership | — (see Architecture §46) | led_number_morph |
| Spring SPRING_K=9/DAMP=5.5 per segment | — (see Architecture §46) | led_number_morph |
| Greedy NN assignment + smoothstep LERP | — (see Architecture §47) | particle_number_morph |
| Origin snapshot ox/oy at morph start | — (see Architecture §47) | particle_number_morph |
| Mandelbrot precompute + live Julia split | — (see Architecture §48) | julia_explorer |
| Auto-wander ellipse orbit c-param | — (see Architecture §48) | julia_explorer |
| IFS chaos game + log-density accumulator | COLOR §24 | barnsley |
| Aspect-corrected launch circle | — (see Architecture §49) | diffusion_map |
| Eden frontier scan toggle | — (see Architecture §50) | diffusion_map |
| Gauss-Seidel φ + phi^eta frontier growth | — (see Master §T3) | tree_la |
| Lyapunov exponent alternating logistic map | — (see Master §T2) | lyapunov |
| int8_t signed bucket encoding | — (see Master §T2) | lyapunov |
| Dual-palette signed value (blue/red) | COLOR §25 | lyapunov |
| Barnes-Hut quadtree s/d<θ criterion | — (see Master §S1) | barnes_hut |
| Static node pool O(1) reset | — (see Master §S3) | barnes_hut |
| Keplerian orbit initialization | — (see Master §S4) | barnes_hut |
| Brightness accumulator DECAY=0.93 | — (see Master §S5) | barnes_hut |
| Speed-normalized body color v/v_max | — (see Master §S6, COLOR §26) | barnes_hut |
| Anchor body (central BH, no integration) | — (see Architecture §44) | barnes_hut |
| Quadtree overlay ACS_HLINE/VLINE depth≤3 | — (see Architecture §44) | barnes_hut |

---

### wave_interference.c

**Analytic superposition of N point sources.** Phase `g_phase[s][r][c] = k·dist(src_s, cell)` is precomputed. Each frame evaluates `sum = Σ cos(phase − ωt)` and maps the signed result to an 8-level ramp (CP_N3..CP_P3). No FDTD — purely analytic.

**Key techniques:** Phase precomputation (§T1), signed 8-level ramp (COLOR §23), CELL_W=8/CELL_H=16 in distance calculation for pixel-space accuracy.

---

### led_number_morph.c

**7-segment LED with particle physics.** Each segment owns exactly N_PER_SEG=24 particles permanently. Particles spring toward their segment's target positions (SPRING_K=9, DAMP=5.5, ζ≈0.92 slightly underdamped). On digit change, segments are toggled on/off; idle particles spring to a parking position off-screen.

**Key techniques:** Bitmask `k_seg_mask[10]` encodes which of 7 segments are lit per digit. Orientation-aware character selection: horizontal segments use `-`, vertical use `|`. Spring force: `F = k·(target−pos) − damp·vel`.

---

### particle_number_morph.c

**Bitmap font morphing via greedy nearest-neighbour LERP.** A 9×7 binary font is expanded to a sub-grid of particles (up to 3×4 per font pixel, ≤500 total). On digit change, each target position is assigned the closest unassigned particle (greedy NN, O(n²)). Positions are then linearly interpolated with `smoothstep(t) = t²(3−2t)`. Origin `ox/oy` is snapshotted at assignment time so interrupted morphs start from wherever each particle currently is.

**Key techniques:** Greedy NN assignment, smoothstep easing, origin snapshot (see Architecture §47), hold timer fires only after morph_t≥1.0.

---

### julia_explorer.c

**Interactive split-screen Julia/Mandelbrot.** Left panel holds a precomputed Mandelbrot buffer `g_mbuf[80][150]`. Right panel recomputes the Julia set every frame for the current c-parameter (crosshair position). Arrow keys / HJKL move the crosshair; auto-wander orbits the Mandelbrot boundary ellipse at WANDER_R=0.72.

**Key techniques:** Precomputed static buffer for the expensive Mandelbrot set, per-frame Julia recompute, dynamic panel widths `g_mw/g_jw` that recompute on resize.

---

### barnsley.c

**IFS chaos game with log-density rendering.** 5 presets (Barnsley Fern, Sierpinski, Lévy C, Dragon, Fractal Tree) each define 4–6 affine transforms with cumulative probability weights. An LCG selects transforms; hits accumulate in a uint32_t grid; `log(1+count)/log(1+max)` normalises the range; 4 characters `. : + @` map to the normalised levels.

**Key techniques:** Cumulative probability transform selection, log-density normalisation (COLOR §24), y-flip in grid mapping (IFS coordinates are bottom-up), hit cap at 60,000 to prevent integer overflow.

---

### diffusion_map.c

**DLA with Eden toggle.** Particles launch from a circle of radius `R_launch = 1.1×max_extent`, walk randomly, and stick when adjacent to the aggregate. Eden mode bypasses the random walk and samples the frontier directly. Age at freeze time is stored in `uint16_t age[][]`; modulo 65536 wraps to create a cyclic gradient across the structure.

**Key techniques:** Aspect-corrected launch (`x=cos·R, y=sin·R·0.5` for screen-circular launch), kill radius at `2×R_launch`, Eden O(rows×cols) frontier scan, age-gradient coloring.

---

### tree_la.c

**DBM Laplacian fractal growth.** Gauss-Seidel iteration solves Laplace's equation (`φ[r][c] = 0.25·(N+S+E+W)`) for the voltage field. Growth probability ∝ `φ^η` at each frontier cell. `η=1` gives DLA-like diffuse trees; `η=4` gives sharp lightning-like branches. Neumann boundary conditions at side walls prevent artificial reflection.

**Key techniques:** Gauss-Seidel relaxation, `φ^η` weighted frontier selection, Neumann BCs `φ[r][0]=φ[r][1]`, φ immediately zeroed after cell addition.

---

### lyapunov.c

**Lyapunov fractal via alternating logistic map.** Each pixel `(a,b)` computes the Lyapunov exponent of the logistic map alternating between parameters a and b. `λ<0` (stable) → blue gradient; `λ>0` (chaotic) → red gradient. Progressive row rendering gives visual feedback during computation.

**Key techniques:** `int8_t` signed bucket encoding (×32 scale), dual-palette signed value (COLOR §25), b-axis inverted (top=low b, bottom=high b) to match standard Lyapunov fractal orientation.

---

### barnes_hut.c

**Barnes-Hut O(N log N) gravity with dual-layer rendering.** A quadtree is rebuilt every tick from a static pool; force traversal applies the `s/d < θ=0.5` opening angle criterion. Rendering has two layers: (1) a brightness accumulator glow (DECAY=0.93) that persists orbital trails, and (2) direct body glyphs colored by `spd/v_max`. Body 0 in the galaxy preset is a fixed central anchor.

**Key techniques:** Static node pool O(1) reset, incremental COM update, Keplerian initialization, dual-layer rendering (glow + direct), rolling v_max with slow decay, quadtree overlay at depth≤3 (ACS_HLINE/ACS_VLINE/ACS_PLUS), `anchor` flag to skip integration on the central BH.

---

*This document is the single reference for all ncurses techniques used across the C files in this project. For fractal algorithms and IFS theory, see Master.md §P and §Q. For the overall loop architecture and subsystem details, see Architecture.md. For color-specific techniques, see COLOR.md.*
