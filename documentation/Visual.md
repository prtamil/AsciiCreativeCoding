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

### V2 — Double Buffer Mechanics
- [V2.1 `curscr` / `newscr` — The Built-in Buffer Pair](#v21-curscr--newscr--the-built-in-buffer-pair)
- [V2.2 `erase()` vs `clear()` — Never Use `clear()` in a Loop](#v22-erase-vs-clear--never-use-clear-in-a-loop)
- [V2.3 `wnoutrefresh` + `doupdate` — The Two-Phase Flush](#v23-wnoutrefresh--doupdate--the-two-phase-flush)
- [V2.4 Why Manual Front/Back Windows Break the Diff Engine](#v24-why-manual-frontback-windows-break-the-diff-engine)
- [V2.5 Framebuffer-to-ncurses: `cbuf` → `fb_blit()`](#v25-framebuffer-to-ncurses-cbuf--fb_blit)

### V3 — Color System
- [V3.1 Color Pairs — `init_pair` / `COLOR_PAIR`](#v31-color-pairs--init_pair--color_pair)
- [V3.2 256-Color vs 8-Color Fallback Pattern](#v32-256-color-vs-8-color-fallback-pattern)
- [V3.3 xterm-256 Palette Index Reference](#v33-xterm-256-palette-index-reference)
- [V3.4 `use_default_colors()` + Background `-1`](#v34-use_default_colors--background--1)
- [V3.5 Grey Ramp for Luminance Gradients](#v35-grey-ramp-for-luminance-gradients)
- [V3.6 `attr_t` — Combining Pair and Attribute Flags](#v36-attr_t--combining-pair-and-attribute-flags)
- [V3.7 A_BOLD / A_DIM as Brightness Tiers](#v37-a_bold--a_dim-as-brightness-tiers)
- [V3.8 Dynamic Color Update — Cosine Palette (flocking.c)](#v38-dynamic-color-update--cosine-palette-flockingc)

### V4 — Character Output
- [V4.1 `mvwaddch` — The Core Write Call](#v41-mvwaddch--the-core-write-call)
- [V4.2 `(chtype)(unsigned char)` Double Cast](#v42-chtypeunsigned-char-double-cast)
- [V4.3 `wattron` / `wattroff` vs `attron` / `attroff`](#v43-wattron--wattroff-vs-attron--attroff)
- [V4.4 `mvprintw` for HUD Text](#v44-mvprintw-for-hud-text)
- [V4.5 ACS Line-Drawing Characters](#v45-acs-line-drawing-characters)
- [V4.6 Paul Bourke ASCII Density Ramp](#v46-paul-bourke-ascii-density-ramp)
- [V4.7 Directional Characters — Velocity to Glyph](#v47-directional-characters--velocity-to-glyph)
- [V4.8 Branch/Slope Characters — Angle to `/`, `\`, `|`, `-`](#v48-branchslope-characters--angle-to----)

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

*This document captures every ncurses-specific pattern from all 24 C files in this repository. For the simulation algorithms and physics, see Master.md. For the overall loop architecture, see Architecture.md.*
