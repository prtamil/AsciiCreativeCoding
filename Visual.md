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

```c
attr_t attr = COLOR_PAIR(p->hue);
if (p->life > 0.65f) attr |= A_BOLD;
```

`attr_t` is ncurses' attribute type. A combined attribute is built by bitwise OR of `COLOR_PAIR(n)` with zero or more attribute flags (`A_BOLD`, `A_DIM`, `A_REVERSE`, etc.). The combined value is passed to `attron()` / `wattron()` in one call rather than multiple calls.

Building the attr into a variable before the draw call keeps the rendering code clean and allows conditional attribute logic (e.g., `A_BOLD` only when `life > 0.65`).

*Files: `brust.c`, `aafire_port.c`, `constellation.c`*

---

#### V3.7 A_BOLD / A_DIM as Brightness Tiers

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

```c
mvwaddch(w, row, col, (chtype)(unsigned char)ch);
```

`mvwaddch(window, y, x, char)` — note the `y, x` order (row first, then column), not `x, y`. Every coordinate in ncurses is (row, col) = (y, x). Getting this backwards produces mirrored/transposed scenes.

`mvwaddch` writes to a specific window. `mvaddch` (no `w`) writes to `stdscr`. Both are used in this project — `w`-variants appear in scene draw functions that accept a `WINDOW*` parameter; plain variants appear in functions that always use `stdscr`.

*Files: all files*

---

#### V4.2 `(chtype)(unsigned char)` Double Cast

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

```c
typeahead(-1);
```

By default ncurses interrupts its output stream mid-flush to poll stdin for pending input. On fast terminals (or when many cells change), this fragments the `doupdate()` write into multiple small writes, causing visible tearing — the terminal shows a partial frame between flushes.

`typeahead(-1)` disables the poll: ncurses writes the entire diff atomically. Input is still checked on the next `getch()` call — no events are lost, just the interrupt.

*Files: all files*

---

#### V6.4 `noecho()` — Suppress Key Echo

```c
noecho();
```

Prevents typed characters from appearing in the terminal as the user presses keys. Without this, pressing `q` during the animation would print the letter `q` to the terminal, corrupting the display.

*Files: all files*

---

#### V6.5 `curs_set(0)` — Hide the Cursor

```c
curs_set(0);
```

Without this, the terminal cursor is left at the position of the last `mvaddch` call — it flickers visibly as it jumps around the screen every frame, distracting from the animation.

`curs_set(0)` hides it entirely. `curs_set(1)` would show it again on exit (not needed since `endwin()` restores terminal state).

*Files: all files*

---

### V7 — Signal Handling & Resize

#### V7.1 `SIGWINCH` — Terminal Resize Signal

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
