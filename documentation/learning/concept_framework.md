# Concept: Framework (Terminal Rendering Engine)

## Pass 1 — Understanding

### Core Idea
A reusable ncurses scaffolding that every simulation in this repo is built on. It handles: terminal setup/teardown, fixed-timestep loop, resize handling, color initialization, and key input. All simulations share this pattern.

### Mental Model
Think of it as the game engine that every simulation plugs into. Instead of writing `initscr()` and `getch()` in every file, you have a proven template. The simulation only fills in the physics and draw functions.

### The Standard Sections
Every file follows this sectioned structure:
```
§1 config     — #defines and enums for all tunable constants
§2 clock      — clock_ns(), clock_sleep_ns()
§3 color      — color_init() with 256-color palette
§4 physics    — simulation state, init, step
§5 draw       — scene_draw() using ncurses primitives
§6 app        — main(): setup, loop, cleanup
```

### Key ncurses Calls
| Call | Purpose |
|------|---------|
| `initscr()` | initialize terminal |
| `cbreak()` | no line buffering |
| `noecho()` | don't echo keystrokes |
| `nodelay(stdscr, TRUE)` | non-blocking getch() |
| `curs_set(0)` | hide cursor |
| `start_color()` | enable color |
| `use_default_colors()` | allow -1 background |
| `init_pair(n, fg, bg)` | define color pair |
| `attron(COLOR_PAIR(n))` | activate color |
| `erase()` | clear screen |
| `wnoutrefresh(stdscr)` | stage update |
| `doupdate()` | apply staged update |

### Physics Coordinate System
- Physics uses floating-point pixel space: `px = col * CELL_W`, `py = row * CELL_H`
- CELL_W=8, CELL_H=16 (typical terminal cell aspect ratio)
- World→screen: `col = (int)(px / CELL_W)`, `row = (int)(py / CELL_H)`
- This makes physics independent of terminal size while staying proportional

### Non-Obvious Decisions
- **`erase()` not `clear()`**: `erase()` marks the virtual screen dirty without writing blanks; `clear()` forces a full repaint. Use `erase()` for flicker-free animation.
- **`wnoutrefresh` + `doupdate`**: Two-phase refresh. `wnoutrefresh` updates the internal model; `doupdate` does one terminal write. Avoids flickering from multiple write calls.
- **`typeahead(-1)`**: Disables ncurses' typeahead detection, which can cause laggy input.
- **Fixed timestep**: Cap `dt_ns` at 100ms to prevent spiral-of-death when window is resized or process is slow.
- **SIGWINCH handler**: Terminal resize sends SIGWINCH. Set flag, then call `endwin(); refresh(); getmaxyx()` to adapt.

## From the Source

**Algorithm:** Reference framework template demonstrating the canonical pattern used by all animations in this project: fixed-step physics accumulator + render interpolation.

**Data-structure:** Fixed-step accumulator: `sim_accum += dt` each frame; drain in SIM_TICK_NS steps: `while (accum ≥ tick) { sim_tick(); accum -= tick; }`. This decouples physics rate from render rate — physics always runs at the same speed regardless of CPU or render load.

**Rendering:** Sub-tick interpolation: `alpha = sim_accum / tick_ns ∈ [0, 1)`. Entity draw positions lerp between prev and current simulated positions at alpha, giving smooth motion at any render rate without modifying physics.

**Performance:** ncurses double-buffer: `erase → draw → wnoutrefresh → doupdate()`. `doupdate()` sends only changed cells to the terminal (diff), minimising write latency and flicker. Render capped at TARGET_FPS using CLOCK_MONOTONIC sleep.

---

### Key Constants
| Name | Value | Role |
|------|-------|------|
| CELL_W | 8 | pixel width of one terminal column |
| CELL_H | 16 | pixel height of one terminal row |
| RENDER_NS | 1e9/60 | frame budget in nanoseconds |
| SIM_DT | 1/60 or 1/120 | simulation timestep |

### Open Questions
- Why does `doupdate()` prevent flickering but `refresh()` alone doesn't?
- What is the minimum terminal size this framework handles gracefully?
- How would you add a second window (panel) for a side HUD?

---

## Pass 2 — Implementation

### Standard Main Structure
```c
int main(void) {
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);
    sim_init();

    long long last = clock_ns();
    while (!g_quit) {
        if (g_resize) { handle_resize(); }
        handle_input(getch());

        long long now = clock_ns();
        long long dt_ns = now - last; last = now;
        if (dt_ns > 100000000LL) dt_ns = 100000000LL;

        if (!g_paused) sim_step((float)dt_ns * 1e-9f);

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
```

### Color Pair Convention
```
CP_1..CP_N   — simulation-specific palette
CP_HUD       — gray text for status line
CP_WARN      — red for out-of-bounds warnings
```

### Build Template
```bash
gcc -std=c11 -O2 -Wall -Wextra <file>.c -o <name> -lncurses -lm
```
