# The Terminal Animation Framework
### A Complete Technical Guide for Beginner Programmers

Based on `ncurses_basics/framework.c` — the canonical template for every animation in this project.

---

## Who This Document Is For

You just wrote your first C program. You understand variables, loops, functions, and maybe structs. Now you want to make something move on the terminal screen. You look at `framework.c` and see 860 lines split into 8 numbered sections, signal handlers, nanosecond clocks, accumulator loops, interpolation factors — and it feels overwhelming.

This document explains every single piece. Not just *what* it does, but *why* it exists, *what breaks* if you remove it, and *how* it connects to the pieces around it. Read it top to bottom once. Then open `framework.c` and read it again alongside this guide. By the end you will understand not just this file, but the architecture used by every one of the 170+ animations in this project.

---

## Table of Contents

1. [The Big Picture — What Is a Game Loop?](#1-the-big-picture--what-is-a-game-loop)
2. [Why Eight Sections? The Separation Principle](#2-why-eight-sections--the-separation-principle)
3. [§1 Config — The Single Source of Truth](#3-1-config--the-single-source-of-truth)
4. [§2 Clock — Measuring Real Time](#4-2-clock--measuring-real-time)
5. [§3 Color — The ncurses Color System](#5-3-color--the-ncurses-color-system)
6. [§4 Coords — Two Worlds, One Bridge](#6-4-coords--two-worlds-one-bridge)
7. [§5 Entity — The Thing Being Simulated](#7-5-entity--the-thing-being-simulated)
8. [§6 Scene — The Container](#8-6-scene--the-container)
9. [§7 Screen — The Display Layer](#9-7-screen--the-display-layer)
10. [§8 App — The Glue and The Loop](#10-8-app--the-glue-and-the-loop)
11. [The Main Loop — Step By Step](#11-the-main-loop--step-by-step)
12. [Fixed Timestep — The Most Important Idea](#12-fixed-timestep--the-most-important-idea)
13. [Render Interpolation — Smooth Motion Without Cheating](#13-render-interpolation--smooth-motion-without-cheating)
14. [Why Separate Simulation from Rendering?](#14-why-separate-simulation-from-rendering)
15. [Two Delta Times — Not a Mistake](#15-two-delta-times--not-a-mistake)
16. [The ncurses Double Buffer](#16-the-ncurses-double-buffer)
17. [Signal Handling — Graceful Shutdown](#17-signal-handling--graceful-shutdown)
18. [App / Screen / Scene — Why Three Structs?](#18-app--screen--scene--why-three-structs)
19. [The Frame Cap — Sleep Before Render](#19-the-frame-cap--sleep-before-render)
20. [Putting It All Together — The Full Data Flow](#20-putting-it-all-together--the-full-data-flow)
21. [How to Write Your Own Animation Using This Framework](#21-how-to-write-your-own-animation-using-this-framework)
22. [Common Mistakes and What They Look Like](#22-common-mistakes-and-what-they-look-like)

---

## 1. The Big Picture — What Is a Game Loop?

Every interactive animation — a video game, a physics simulation, a visualisation — is built on the same fundamental pattern: the **game loop**. It is an infinite loop that runs as fast as the hardware allows (or as fast as you tell it to), and in each iteration it does exactly three things:

```
while (running) {
    update physics / simulation
    render the frame
    handle input
}
```

That is it. Three things. Over and over, dozens of times per second.

The challenge is that these three things interfere with each other in subtle ways. If updating takes too long one frame, the render is late. If the terminal is slow, drawing takes longer, and the physics falls behind. If physics and rendering are coupled together — if you run one physics step per rendered frame — then your simulation runs at different speeds on different computers, or even on the same computer under different load.

`framework.c` solves all of these problems. It is not just three lines in a loop. It is a carefully designed machine with distinct, separated concerns. The 8 sections are not arbitrary — each one owns exactly one responsibility and has no knowledge of the others.

---

## 2. Why Eight Sections? The Separation Principle

Before diving into the code, understand *why* it is split the way it is.

The core principle is: **things that change for different reasons should be in different places.**

| Section | Responsibility | Changes when... |
|---------|---------------|-----------------|
| §1 config | Tunable constants | You want different behaviour |
| §2 clock | Time measurement | You change OS or need higher precision |
| §3 color | Terminal color setup | You add new visual themes |
| §4 coords | Pixel↔cell mapping | You change the coordinate model |
| §5 entity | Simulation logic | You change what you are simulating |
| §6 scene | Entity collection | You add more entities |
| §7 screen | ncurses rendering | You change how things are displayed |
| §8 app | Loop, signals, input | You change the overall control flow |

This means when you want to add a new entity type, you only touch §5. When you want a new color scheme, you only touch §3. When you want to fix a rendering bug, you only look in §7.

**What happens if you don't separate?**

If you mix physics update and drawing in the same function, every change to how something moves requires touching the drawing code, and vice versa. The file becomes spaghetti. In a 300-line file this is annoying. In an 800-line file it is a maintenance nightmare. In a project with 170 files, every one of which a different student might study, it becomes impossible to understand.

The sections also create a **dependency order**:

```
§1 ← §2, §3, §4, §5, §6, §7, §8   (everything reads config)
§2 ← §8                              (app uses clock)
§3 ← §7                              (screen uses color)
§4 ← §6                              (scene uses coords)
§5 ← §6                              (scene contains entity)
§6 ← §7, §8                          (screen/app use scene)
§7 ← §8                              (app uses screen)
```

Lower-numbered sections never call higher-numbered ones. This is a **layered architecture** — a pattern you will see in every well-designed software system for the rest of your career.

---

## 3. §1 Config — The Single Source of Truth

```c
enum {
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    HUD_COLS         =  48,
    FPS_UPDATE_MS    = 500,
    N_COLORS         =   7,
    KEY_LEN          =  26,
};

#define ASCII_FIRST  0x21
#define ASCII_LAST   0x7E
#define ASCII_RANGE  (ASCII_LAST - ASCII_FIRST + 1)

#define RATE_MIN   4.0f
#define RATE_MAX  28.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))
```

Every number that controls behaviour lives here and *only* here. There are no magic numbers anywhere else in the file.

**Why `enum` instead of `#define` for integer constants?**

An `enum` constant is a named value in the compiler's symbol table. You can see it in a debugger, it participates in type checking, and you cannot accidentally redefine it. A `#define` is a text substitution — the preprocessor replaces `SIM_FPS_DEFAULT` with `60` before the compiler even sees the code. The compiler never knows the name `SIM_FPS_DEFAULT` existed. For integer constants in C, `enum` is generally preferred.

**Why `#define` for the floating-point constants?**

`RATE_MIN` and `RATE_MAX` are `float` values (note the `f` suffix). C `enum` only supports integer types. So `#define` is the right tool here.

**Why `#define` for `TICK_NS(f)`?**

`TICK_NS` is a *macro* — a function-like expression that the preprocessor expands inline. `TICK_NS(app->sim_fps)` expands to `(NS_PER_SEC / (app->sim_fps))`. This is appropriate because it involves a runtime variable (`app->sim_fps`) that changes when the user presses `[` or `]`.

**What happens if you scatter magic numbers through the code?**

Suppose you write `while (sim_accum >= 16666666)` on line 800, `int tick_ms = 16` on line 200, and `sleep_ns(16666666)` on line 850. These three numbers are the same concept: 1/60th of a second. If you decide to change from 60 fps to 30 fps, you have to hunt for all three, know they are related, and change each one. Miss one and you get a subtle timing bug that is very hard to find. With `TICK_NS(60)` defined once, you change one line and everything updates.

**The `NS_PER_SEC` / `NS_PER_MS` naming:**

These are `long long` constants (note the `LL` suffix). Nanoseconds in a second: 1,000,000,000. That does not fit in a 32-bit `int` (max ~2.1 billion without overflow risk). The `LL` suffix ensures the constant is treated as 64-bit from the start, preventing overflow when it is multiplied.

---

## 4. §2 Clock — Measuring Real Time

```c
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}
```

### Why nanoseconds?

A single animation frame at 60fps lasts approximately 16,666,666 nanoseconds (16.7 milliseconds). Physics steps can be even shorter. If you measured time in milliseconds with `int`, you would have only 16 distinct values per frame — not enough precision for smooth timing. Nanoseconds give you 16,666,666 distinct values per frame. The precision is 1000× better.

### Why `CLOCK_MONOTONIC` and not `time(NULL)` or `gettimeofday`?

There are several clocks available on a Linux system:

| Clock | What it measures | Problem |
|-------|-----------------|---------|
| `time(NULL)` | Seconds since epoch | Only 1-second precision |
| `gettimeofday` | Microseconds since epoch | Can jump backwards on NTP sync |
| `CLOCK_REALTIME` | Nanoseconds, wall clock | Can jump backwards on NTP sync |
| `CLOCK_MONOTONIC` | Nanoseconds, always increasing | **This is what we want** |

`CLOCK_MONOTONIC` is guaranteed by POSIX to never go backwards. It starts at some arbitrary point (usually system boot) and counts forward. It does not care about timezone changes, NTP adjustments, or daylight saving time.

**What happens if you use `CLOCK_REALTIME`?**

On a laptop that syncs time with an NTP server, `CLOCK_REALTIME` can jump backwards by several hundred milliseconds. Your `dt = now - frame_time` suddenly becomes a large negative number. `sim_accum` goes negative, the accumulator loop never fires, physics stops for hundreds of frames, then suddenly fires thousands of times in a row to catch up. The animation stutters violently or freezes. With `CLOCK_MONOTONIC` this cannot happen.

### Why `int64_t` and not `double`?

`double` has 53 bits of mantissa. A nanosecond timestamp at runtime (seconds since boot, say 10,000 seconds = 10^13 nanoseconds) exceeds 53 bits. The precision of a `double` at 10^13 is about 2048 nanoseconds — worse than 2 microseconds. Two consecutive `clock_ns()` calls on a fast CPU could return the same `double` even though they are 100 ns apart.

`int64_t` is a 64-bit signed integer. It can represent values up to about 9.2 × 10^18 nanoseconds — that is 292 years. Every nanosecond is exactly representable.

### Why `static`?

Functions and variables marked `static` at file scope are invisible outside this translation unit (this `.c` file). This means if another `.c` file also defines a `clock_ns()` function (which the other files in this project do), there is no name collision. Each file has its own private copy.

**What happens if you remove `static`?**

When you link multiple `.c` files together, the linker sees two `clock_ns` symbols and throws a "multiple definition" error. Every file in this project has its own `clock_ns` — `static` is what makes that work.

### The `clock_sleep_ns` early-return guard

```c
if (ns <= 0) return;
```

If the physics step took longer than the frame budget (busy machine, large simulation, slow terminal), the remaining sleep time is negative. Calling `nanosleep` with a negative time is undefined behaviour on some systems and sleeps for a very long time on others. The guard makes the "we are over budget, skip the sleep" case explicit and safe.

---

## 5. §3 Color — The ncurses Color System

```c
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);   /* red     */
        init_pair(2, 208, COLOR_BLACK);   /* orange  */
        init_pair(3, 226, COLOR_BLACK);   /* yellow  */
        init_pair(4,  46, COLOR_BLACK);   /* green   */
        init_pair(5,  51, COLOR_BLACK);   /* cyan    */
        init_pair(6,  21, COLOR_BLACK);   /* blue    */
        init_pair(7, 201, COLOR_BLACK);   /* magenta */
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        /* ... */
    }
}
```

### What is a color pair?

ncurses cannot set foreground and background colors independently per character. Instead, it uses **color pairs** — a single integer that encodes both a foreground and a background color. You define pairs once at startup with `init_pair(id, fg, bg)`, then apply them at draw time with `COLOR_PAIR(id)`.

Pair 0 is reserved by ncurses for the default terminal colors. Your pairs start at 1.

### Why `start_color()` first?

ncurses does not enable color support automatically. `start_color()` initialises the color subsystem, allocates internal tables, and enables the `COLOR_PAIR` and `init_pair` functions. Calling `init_pair` before `start_color` silently does nothing — no error, no crash, just invisible colors.

### Why `use_default_colors()`?

Without this call, every color pair must have an explicit foreground and background. The terminal's own default background (which might be a custom color, or transparent in a compositing terminal) cannot be used. `use_default_colors()` adds a special value `-1` meaning "terminal default" for both fg and bg. This makes programs look correct on terminals with custom themes — the background shows through instead of a hardcoded black square.

**What happens if you remove it?**

On a terminal with a white background, `COLOR_BLACK` background gives characters a black rectangle that looks wrong. With `use_default_colors()` you can use `-1` as the background and let the terminal's own color show through.

### The `COLORS >= 256` check

`COLORS` is a global integer set by ncurses at `initscr()` time. It reflects what the terminal's `$TERM` environment variable reports about its capabilities. A standard xterm-256color terminal gives `COLORS = 256`. An SSH connection to an old server might give `COLORS = 8`.

The check makes the program work on both. On 256-color terminals you get rich saturated palette colors (indices like 196 for a vivid red, 46 for a vivid green). On 8-color terminals you fall back to the named `COLOR_RED`, `COLOR_GREEN` constants.

**What happens if you skip the check and always use 256-color indices?**

On an 8-color terminal, `init_pair(1, 196, COLOR_BLACK)` silently clamps the index to the nearest available color. You get unpredictable and usually wrong colors. Some terminals treat out-of-range indices as wrapping (196 % 8 = 4 = blue on many terminals) — your red pair becomes blue.

### xterm-256 color layout

The 256 xterm colors are arranged as:
- 0–15: standard 16 colors (black, red, green, yellow, blue, magenta, cyan, white × normal + bright)
- 16–231: 6×6×6 RGB color cube (index = 16 + 36r + 6g + b where r,g,b ∈ [0,5])
- 232–255: 24-step greyscale ramp from near-black to near-white

Index 196 = 16 + 36×5 + 6×0 + 0 = 16 + 180 = 196 → the maximum-red cube entry. That is why 196 is vivid red, 46 is vivid green, 21 is vivid blue.

---

## 6. §4 Coords — Two Worlds, One Bridge

This is one of the most important and most misunderstood sections. Read it carefully.

### The problem: terminal cells are not square

A typical terminal cell is 8 pixels wide and 16 pixels tall. The aspect ratio is 1:2 (width:height in physical pixels).

Now suppose you have a ball moving at 45 degrees — equal speed in X and Y. You store its position as floating-point numbers, update `x += speed` and `y += speed` each frame, then draw it at `(int)x, (int)y` on screen.

What do you see? The ball moves at a 45-degree angle in logical cell coordinates. But in physical pixels it moves 8 pixels per cell horizontally and 16 pixels per cell vertically. It appears to move at a 27-degree angle, not 45. Diagonal motion looks wrong. Circles become ellipses. Velocities are anisotropic — the same number means different physical distances in X and Y.

### The solution: pixel space

Define a **pixel space** coordinate system where one unit = one physical pixel (approximately). Then use a simple formula to convert to cell coordinates only when drawing.

```c
#define CELL_W   8    /* pixels per cell, horizontal */
#define CELL_H  16    /* pixels per cell, vertical   */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px) {
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
    return (int)floorf(py / (float)CELL_H + 0.5f);
}
```

Now physics operates in pixel space. A velocity of `(vx=1, vy=1)` means 1 physical pixel per second in both directions — truly equal. A circle traced as `x = cx + r·cos(θ)` in pixel space maps to an actual circle on screen.

The conversion happens **once**, **only in `scene_draw()`**. Physics code never touches cell coordinates. This is the single conversion point.

**What happens if you mix spaces?**

Say you store position in cells and velocity in pixels. Every tick you update `x += vx` where vx is in pixels/tick, but x is in cells — the unit is wrong. The ball moves 8× too fast horizontally. Or you store everything in cells and physics looks fine at 80-column terminals but completely breaks on 200-column terminals because your fixed constants assume a specific terminal width.

Pixel space is independent of terminal size. `pw(cols)` gives the total pixel width based on the *current* terminal size. After a resize, `pw(cols)` automatically gives the new width — no special handling needed.

### Why `floorf(px / CELL_W + 0.5f)` and not `roundf`?

This is a subtle but real bug in naive implementations.

`roundf` uses "round half to even" — when the value is exactly 0.5, it rounds to the nearest even number. So 0.5 rounds to 0, 1.5 rounds to 2, 2.5 rounds to 2, 3.5 rounds to 4. This is called banker's rounding.

For a slowly moving object whose pixel coordinate drifts towards a cell boundary from below, `roundf` might round to one cell on one frame and the opposite cell on the next frame — causing one-pixel flicker as the object sits on the boundary.

`floorf(px / CELL_W + 0.5f)` always rounds 0.5 upward: 0.5→1, 1.5→2, 2.5→3. It is deterministic and breaks ties in one consistent direction. No flicker.

**What about just `(int)(px / CELL_W)`?**

That truncates — always rounds down. A ball at pixel 7.9 maps to cell 0, but a ball at pixel 8.0 maps to cell 1. The ball "snaps" to the next cell when it crosses the boundary, which is correct — but the dwell time at cell 0 is asymmetric: it stays in cell 0 from pixel 0.0 to pixel 7.999, but only reaches cell 1 at exactly 8.0. With `+0.5f` rounding, the dwell time is symmetric: cell 0 from 0.0–3.99, cell 1 from 4.0–11.99, which looks smoother.

### When to skip §4

If your simulation operates directly on cell coordinates — fire spreading through a grid, falling sand, a character rain effect — there is no continuous physics. Positions are already integers (row, col). No conversion is needed. §4 is omitted from such simulations, but it is always kept in the framework template because the moment you add a ball or a particle, you need it.

---

## 7. §5 Entity — The Thing Being Simulated

```c
typedef struct {
    char  slots[KEY_LEN];
    float timers[KEY_LEN];
    float rates[KEY_LEN];
    int   colors[KEY_LEN];
    float speed_scale;
    bool  paused;
} KeyGen;
```

§5 is the only section that changes fundamentally between animations. In `bounce_ball.c` this section contains `Ball` with position, velocity, and radius. In `double_pendulum.c` it contains `DoublePendulum` with angles, angular velocities, and lengths. In `flocking.c` it contains `Boid` with position, velocity, and neighbor lists.

In `framework.c`, the entity is `KeyGen` — a simple example chosen because it has no continuous motion (making it easy to understand the structure without getting distracted by physics).

### The `KeyGen` struct — every field explained

**`slots[KEY_LEN]`** — the 26 characters currently displayed on screen. Each is a printable ASCII character in the range `0x21–0x7E`. The array holds the current state of the key display.

**`timers[KEY_LEN]`** — countdown timers in seconds. When `timers[i]` reaches zero, `slots[i]` gets a new random character and the timer is reset. Each slot has its own independent timer so they do not all flip at the same time.

**`rates[KEY_LEN]`** — the number of character changes per second for each slot. Randomised once at spawn in the range `[RATE_MIN, RATE_MAX]`. A slot with rate 28 changes 28 times per second; a slot with rate 4 changes 4 times per second. This creates the organic-looking "not-all-at-once" effect.

**`speed_scale`** — a global multiplier. When the user presses `+`, `speed_scale *= 1.5f`. The effective rate of slot `i` becomes `rates[i] * speed_scale`. Pressing `-` divides. This scales all slots uniformly while preserving their relative rates.

**`paused`** — when true, `keygen_tick` returns immediately. The simulation state is completely frozen — not just rendering, but actual state updates stop.

### `keygen_spawn()` — initialise or re-randomise

```c
static void keygen_spawn(KeyGen *k)
{
    k->speed_scale = 1.0f;
    k->paused      = false;
    for (int i = 0; i < KEY_LEN; i++) {
        k->slots[i]  = (char)(ASCII_FIRST + rand() % ASCII_RANGE);
        k->rates[i]  = RATE_MIN
                     + ((float)(rand() % 10000) / 10000.0f)
                       * (RATE_MAX - RATE_MIN);
        k->timers[i] = 1.0f / k->rates[i];
        k->colors[i] = (i % N_COLORS) + 1;
    }
}
```

The initial timer value `1.0f / k->rates[i]` is the period — the time between one change and the next at this rate. This seeds the timer to a "ready to fire in one full period" state. If you seeded all timers to 0, every slot would fire immediately on the first tick, then fan out. Seeding to the period means the first changes happen staggered across the first second of animation.

The rate is computed as `RATE_MIN + fraction * (RATE_MAX - RATE_MIN)`. The `rand() % 10000 / 10000.0f` gives a float uniformly in [0, 1). Multiplying by the range and adding the minimum maps it to [RATE_MIN, RATE_MAX).

### `keygen_tick()` — the physics step

```c
static void keygen_tick(KeyGen *k, float dt)
{
    if (k->paused) return;

    float scaled_dt = dt * k->speed_scale;
    for (int i = 0; i < KEY_LEN; i++) {
        k->timers[i] -= scaled_dt;
        if (k->timers[i] <= 0.0f) {
            k->slots[i] = (char)(ASCII_FIRST + rand() % ASCII_RANGE);
            float interval = 1.0f / (k->rates[i] * k->speed_scale);
            k->timers[i] = (interval > 0.001f) ? interval : 0.001f;
        }
    }
}
```

This is the physics function. It takes one parameter beyond `k`: `dt` — the duration of this timestep in seconds.

**Why `float scaled_dt = dt * k->speed_scale`?**

Multiplying `dt` by `speed_scale` is equivalent to running the simulation at `speed_scale` times normal speed. If `speed_scale = 2.0f`, the timer drains twice as fast — characters change twice as often. The real clock has not changed; the simulation is scaled.

**The timer reset floor `(interval > 0.001f) ? interval : 0.001f`**

If `speed_scale` is very large (say 16×) and `rates[i]` is also at its max (28 changes/sec), the interval becomes `1/(28×16) ≈ 0.002f`. Theoretically fine. But if `speed_scale` is pushed to an extreme beyond what the code allows (future modification), the interval could hit zero or go negative. The 0.001f floor (1 ms minimum interval) prevents division-by-zero and infinite-fire scenarios.

### The critical rule: §5 never calls ncurses

`keygen_tick` does not call `mvwaddch`, `wattron`, or any ncurses function. It only modifies the `KeyGen` struct. The struct is pure data — no rendering, no I/O.

**What happens if you draw from the physics function?**

The physics function is called in a tight loop (potentially multiple times per frame). ncurses drawing in that loop means the display is partially updated mid-frame. You see torn frames: some entities drawn at their old positions, some at their new. The double-buffer (§7) only works correctly when drawing is done once per frame in `scene_draw`.

Additionally, physics and rendering being interleaved means you cannot run physics faster than your render rate, or slower, without visual artifacts. The entire point of separating them is that physics runs at its own rate, completely independently.

---

## 8. §6 Scene — The Container

```c
typedef struct {
    KeyGen kg;
} Scene;

static void scene_init(Scene *s) {
    memset(s, 0, sizeof *s);
    keygen_spawn(&s->kg);
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
    (void)cols; (void)rows;
    keygen_tick(&s->kg, dt);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    /* ... layout and drawing ... */
}
```

### Why a Scene struct at all?

In a more complex animation you might have:
- 20 bouncing balls
- 5 particle emitters
- A background grid
- A score counter

The scene owns all of them. `scene_tick` calls each entity's tick function. `scene_draw` calls each entity's draw function. The main loop always calls `scene_tick` and `scene_draw` — it never needs to know what is inside.

If you later add a second entity type (say, a static title), you add it to `Scene`, call its tick/draw from `scene_tick`/`scene_draw`, and the main loop does not change. This is the **closed-for-modification, open-for-extension** principle.

### `memset(s, 0, sizeof *s)`

Zeroes every byte of the Scene struct before initialization. This is a defensive practice: it ensures no field contains garbage from uninitialized stack memory. `float` zeroed bytes = `0.0f`. `int` zeroed = `0`. `bool` zeroed = `false`. `pointer` zeroed = `NULL`. Even if you forget to explicitly initialize a field, it starts in a known safe state.

**What happens without the memset?**

Stack-allocated structs in C have undefined contents before initialization. If you forget to set `k->speed_scale`, it might contain whatever was on the stack before — perhaps 2.7×10^38 (a garbage float). On first tick, every slot fires approximately 2.7×10^38 times and the program freezes.

### Why does `scene_tick` take `cols` and `rows`?

In animations with pixel-space physics (like `bounce_ball.c`), `scene_tick` calls `pw(cols)` and `ph(rows)` to get the pixel-space boundaries and clamp the ball's position. The terminal might have been resized; the current dimensions must be passed each tick.

`KeyGen` does not use pixel space, so `cols` and `rows` are cast to `(void)` to silence "unused parameter" compiler warnings. The parameters are still present because the **signature is fixed across all animations**. If you call `scene_tick` from the main loop, you always pass `(s, dt, cols, rows)` regardless of what the specific animation does with them. Uniformity means the main loop template never changes.

### `scene_draw` — the one drawing function

```c
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
```

Note: `const Scene *s`. The scene is read-only during drawing. Drawing never modifies simulation state. This is enforced by the compiler — if you accidentally try to change a field of `s` inside `scene_draw`, the compiler gives a `"discards const qualifier"` error.

`WINDOW *w` is the ncurses window to draw into. For single-window programs this is always `stdscr`. Passing it as a parameter (rather than using `stdscr` directly) means the function could theoretically draw into a sub-window or a different window without modification — a small but useful design flexibility.

`alpha` and `dt_sec` are the interpolation parameters — explained in depth in section 13.

---

## 9. §7 Screen — The Display Layer

```c
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s) { ... }
static void screen_free(Screen *s) { ... }
static void screen_resize(Screen *s) { ... }
static void screen_draw(Screen *s, const Scene *sc, ...) { ... }
static void screen_present(void) { ... }
```

### What does Screen own?

Screen owns the ncurses session and the current terminal dimensions. It is the only section that calls `initscr()`, `endwin()`, `erase()`, `wnoutrefresh()`, and `doupdate()`.

Every other section is ignorant of ncurses internals. §5 does not know what `erase()` does. §6 knows it can call `mvwaddch` but does not know when. §7 decides when the frame is built and when it is pushed to the terminal.

### `screen_init` — the full ncurses setup sequence

```c
static void screen_init(Screen *s)
{
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
```

Every single call matters. Here is what each one does and what breaks without it:

**`initscr()`** — the entry point for ncurses. Allocates internal data structures, reads terminal capabilities from `$TERM`, sets up `stdscr`. Without this, no other ncurses call works. Call it first, before anything ncurses-related.

**`noecho()`** — by default, every character you press on the keyboard is echoed to the terminal. In a game, pressing `q` would print `q` on screen before your code even sees it. `noecho()` disables this. Without it, your key presses leave visible garbage on the display.

**`cbreak()`** — by default, the terminal buffers input until you press Enter (canonical mode). A game needs to respond to individual keypresses without waiting for Enter. `cbreak()` passes each keypress immediately to `getch()`. Without it, you press `q` and nothing happens until you press Enter — unacceptable for real-time interaction.

**`curs_set(0)`** — hides the hardware blinking cursor. Without this, a cursor blinks at position (0,0) or wherever your last `mvwaddch` left it, which is visually distracting and makes the animation look amateur.

**`nodelay(stdscr, TRUE)`** — makes `getch()` non-blocking. Normally `getch()` waits indefinitely for a keypress. In a game loop, you cannot wait — you must poll for input and continue the loop if nothing is pressed. With `nodelay(TRUE)`, `getch()` returns `ERR` immediately if no key is pending. Without this, the animation freezes waiting for you to type something.

**`keypad(stdscr, TRUE)`** — enables ncurses' translation of multi-byte escape sequences into named constants. Arrow keys, function keys, and others send multiple bytes to the terminal. Without `keypad(TRUE)`, pressing the up-arrow key gives you three raw characters `ESC [ A` instead of the single constant `KEY_UP`. Most of the programs in this project use arrow keys for control.

**`typeahead(-1)`** — this is the most subtle one. ncurses, by default, calls `read()` on stdin mid-draw to check if the user has typed something (to "peek ahead"). If a key is detected, ncurses aborts the current draw operation and processes input first. This causes incomplete frames — the terminal receives a partial update and you see flickering, missing characters, and torn animations. `typeahead(-1)` disables this behavior entirely. Every frame is drawn completely before input is checked. Without this, fast-typing users or slow terminals cause visible frame tearing.

**`getmaxyx(stdscr, s->rows, s->cols)`** — reads the current terminal dimensions into the Screen struct. This is a macro that expands to two assignments. The initial values are needed immediately for the first frame's layout calculations.

### `screen_free` — always called on exit

```c
static void screen_free(Screen *s) {
    (void)s;
    endwin();
}
```

`endwin()` restores the terminal to the state before `initscr()` was called: re-enables echo, restores the cursor, re-enables canonical input mode. Without `endwin()`, when the program exits the terminal is left in raw no-echo mode — you type but see nothing, and the shell prompt behaves strangely until you type `reset`.

The `atexit(cleanup)` call in `main()` registers a second `endwin()` as a safety net that fires even if the program crashes or calls `exit()` unexpectedly.

### `screen_resize` — handling terminal resize

```c
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}
```

When the user resizes the terminal window, the OS sends `SIGWINCH` (Signal Window CHange). The signal handler sets `need_resize = 1`. At the start of the next loop iteration, `app_do_resize` is called, which calls `screen_resize`.

**Why `endwin()` + `refresh()`?**

ncurses caches the terminal dimensions at `initscr()` time. When the terminal is resized, ncurses does not know. Calling `endwin()` resets ncurses' internal state, and `refresh()` forces it to re-query `LINES` and `COLS` from the kernel. The new dimensions are then read with `getmaxyx`.

**What happens without resize handling?**

ncurses still thinks `cols = 80, rows = 24` even though the terminal is now `cols = 120, rows = 40`. Calls to `mvwaddch(row, col)` where `col >= 80` silently fail. Parts of the new larger terminal are blank. Worse, if the user makes the terminal *smaller*, ncurses tries to draw at positions that no longer exist — this causes ncurses to enter an error state and sometimes crashes the program.

### `screen_draw` — building the frame

```c
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD last — always on top */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  spd:%.2fx  %s ",
             fps, sim_fps, sc->kg.speed_scale,
             sc->kg.paused ? "PAUSED " : "running");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
}
```

**`erase()` not `clear()`**

Both blank the screen, but `clear()` marks every cell as "changed" — it forces ncurses to redraw everything next `doupdate()`. `erase()` only blanks the virtual buffer; `doupdate()` still sends only cells that actually changed. For a 120×50 terminal, `clear()` sends 6000 characters every frame. `erase()` sends only the cells that actually moved. At 60fps, this is the difference between ~360,000 characters/sec and ~a few hundred changed cells/sec.

**Draw order: scene first, HUD last**

The HUD (Heads-Up Display — the status text showing fps, mode, keys) is drawn after `scene_draw()`. In ncurses, the last write to a cell wins. If the scene draws at row 0 (where the HUD lives), the HUD overwrites it. The HUD is always visible regardless of what the animation does.

**`snprintf` for HUD text**

`snprintf(buf, sizeof buf, ...)` writes into a local buffer with a size limit. This prevents buffer overflow if the format string produces more characters than `HUD_COLS + 1`. A `mvprintw` with unlimited format could overflow the stack if the values are unexpectedly large.

### `screen_present` — the one flush

```c
static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}
```

**`wnoutrefresh(stdscr)`** — marks `stdscr`'s contents as "ready to send" but does not write to the terminal yet. It copies `stdscr` into ncurses' internal `newscr` model.

**`doupdate()`** — compares `newscr` with `curscr` (what ncurses thinks is currently on screen), computes the minimal set of changes, writes only those changes to the terminal file descriptor, and sets `curscr = newscr`.

**Why two calls? Why not just `refresh()`?**

`refresh()` is equivalent to `wnoutrefresh(stdscr)` + `doupdate()` in one call. For a single-window program the result is the same. But for programs with multiple ncurses WINDOWs, calling `refresh()` on each window individually means multiple `doupdate()` calls — multiple partial frames written to the terminal. With `wnoutrefresh` on all windows followed by a single `doupdate`, the terminal sees exactly one complete update. The habit of using `wnoutrefresh + doupdate` instead of `refresh` is correct even in single-window programs, because it makes the code safe to extend with multiple windows later.

---

## 10. §8 App — The Glue and The Loop

```c
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;
```

### Why a global `g_app`?

Signal handlers in C have a fixed signature: `void handler(int sig)`. They cannot take additional arguments. If `App` is a local variable in `main`, signal handlers have no way to reach it to set `running = 0` or `need_resize = 1`.

The solution is to make `g_app` a global. Signal handlers can then set `g_app.running = 0` directly. Using a global is generally discouraged, but for the signal handler interface there is no clean alternative in C.

There is only one `App` instance in the program. Making it global is safe and explicit — the name `g_app` (g_ prefix signals "global") makes its scope immediately obvious to any reader.

### Why `volatile sig_atomic_t`?

This type is required by the C standard for variables that are both written by a signal handler and read by the main program. Two things are happening:

**`sig_atomic_t`** — guaranteed by the C standard to be read and written atomically (without interruption). On most platforms this is an `int`. The guarantee is that the main loop cannot read `running` in a half-written state if the signal handler is writing it simultaneously.

**`volatile`** — tells the C compiler that this variable may change from outside the program's own control flow. Without `volatile`, an optimising compiler might see that `running` is never written in the main loop body and "optimise" the check `while (app->running)` into an infinite loop with the check removed. With `volatile`, the compiler must re-read the variable from memory on every check, because it knows an external source (the signal handler) might have changed it.

**What happens without `volatile`?**

With `-O2` optimisation (the standard build flag for this project), the compiler may cache `running` in a register at the start of the loop and never re-read it from memory. You press Ctrl+C, the signal handler sets `g_app.running = 0`, but the main loop reads the cached register value (still 1) and never sees the change. The program does not quit. You have to send SIGKILL (`kill -9`) to terminate it. This is a real bug that appears only with optimisation, making it extremely hard to diagnose.

### Signal handlers

```c
static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }
```

**`(void)sig`** — the signal number parameter is not used. Casting it to void silences the "unused parameter" compiler warning. This is idiomatic C.

**Why only set a flag, not act immediately?**

Signal handlers have severe restrictions. They are not allowed to call most library functions (including `malloc`, `printf`, and any ncurses function). A signal handler that calls `endwin()` directly has undefined behaviour — it might work on your system today and crash on a different OS or compiler. The safe pattern is:

1. Signal handler sets a flag (safe — one atomic write)
2. Main loop checks the flag at a safe point (start of iteration)
3. Main loop performs the actual action

For `SIGWINCH` this is particularly important: `screen_resize` calls `endwin()` and `refresh()` — complex ncurses operations that are absolutely not safe in a signal handler.

### `app_handle_key` — input as a state machine

```c
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':  k->paused = !k->paused; break;
    case 'r':  keygen_spawn(k); break;
    case '=': case '+': k->speed_scale *= 1.5f; break;
    case '-':  k->speed_scale /= 1.5f; break;
    case ']':  app->sim_fps += SIM_FPS_STEP; break;
    case '[':  app->sim_fps -= SIM_FPS_STEP; break;
    }
    return true;
}
```

All input handling is in one place. This makes it trivial to add new keys: add a case. To see all keys, look here. The return value is `false` only for quit — a clean "should we continue?" contract with the main loop.

---

## 11. The Main Loop — Step By Step

This is the heart of the framework. Every animation in this project has this identical structure. The specific comments in the code number the steps ①–⑦.

```c
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        /* ① resize */
        /* ② dt */
        /* ③ sim accumulator */
        /* ④ alpha */
        /* ⑤ fps counter */
        /* ⑥ frame cap + sleep */
        /* ⑦ draw + present */
        /* ⑧ input */
    }

    screen_free(&app->screen);
    return 0;
}
```

### Initialization before the loop

**`srand(clock_ns() & 0xFFFFFFFF)`** — seeds the pseudo-random number generator with the lower 32 bits of the current monotonic clock. This makes each run of the program produce a different random sequence. `& 0xFFFFFFFF` masks to 32 bits because `srand` takes `unsigned int` (typically 32-bit). Two programs started at the same nanosecond would get the same seed, but in practice this never happens.

**`frame_time = clock_ns()`** — initialises the last-frame timestamp to right now, so the first `dt` measurement (at the start of the first loop iteration) gives a near-zero elapsed time rather than a garbage large value.

**`sim_accum = 0`** — the accumulator starts empty.

---

## 12. Fixed Timestep — The Most Important Idea

This is the concept that distinguishes a professional game loop from a naive one. Read this section carefully. It is used in every physics simulation, game, and animation in this project.

### The naive approach and why it fails

A beginner might write:

```c
while (running) {
    int64_t now = clock_ns();
    float dt = (float)(now - last_time) / 1e9f;
    last_time = now;

    ball.x += ball.vx * dt;
    ball.y += ball.vy * dt;

    draw();
}
```

This is called a **variable timestep** loop. Physics advances by `dt` seconds each frame, where `dt` is the actual elapsed time. If the frame takes 16ms, `dt = 0.016`. If the frame takes 32ms (slow), `dt = 0.032`.

It sounds correct — you scale the physics by the actual time elapsed. But it has serious problems:

1. **Non-determinism**: Run the program twice and you get different results because the frame times are different. You cannot reproduce a specific trajectory.
2. **Numerical instability**: Physics integrators (Euler, RK4) are stable only for `dt` below a threshold. If a frame is delayed (OS scheduling, background task), `dt` spikes and the simulation explodes.
3. **Different speeds on different hardware**: A fast machine renders 120fps, `dt = 0.008`. A slow machine renders 15fps, `dt = 0.067`. Physics runs at the same real-time speed but with very different accuracy — the simulation looks different on each machine.
4. **Floating-point divergence**: Two balls that should collide at `dt = 0.016` might pass through each other at `dt = 0.032` because the discrete step skips the collision point.

### The fixed timestep accumulator

```c
int64_t tick_ns = TICK_NS(app->sim_fps);      /* e.g. 1/60 sec in ns    */
float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;  /* 0.01666... sec */

sim_accum += dt;   /* add real elapsed time to the bucket */

while (sim_accum >= tick_ns) {
    scene_tick(&app->scene, dt_sec,
               app->screen.cols, app->screen.rows);
    sim_accum -= tick_ns;
}
```

**The accumulator is a time bucket.** Each frame, you pour the real elapsed time `dt` into the bucket. Physics runs in fixed-size scoops of `tick_ns` each. After each scoop, `tick_ns` is removed from the bucket. When the bucket has less than one scoop left, the physics loop stops.

**Example — steady 60fps render, 60Hz sim:**
- Frame 1: dt = 16.7ms. accum = 16.7ms. tick = 16.7ms. One scoop. accum = 0ms.
- Frame 2: dt = 16.7ms. accum = 16.7ms. One scoop. accum = 0ms.
- Steady: one physics tick per render frame.

**Example — slow frame (render took 33ms), 60Hz sim:**
- dt = 33.4ms. accum = 33.4ms. Two scoops. accum = 0ms.
- Physics ran twice this frame to catch up. Still correct.

**Example — fast render (120fps), 60Hz sim:**
- Frame 1: dt = 8.3ms. accum = 8.3ms. Zero scoops (8.3 < 16.7). accum = 8.3ms.
- Frame 2: dt = 8.3ms. accum = 16.6ms. One scoop. accum ≈ 0ms.
- Physics runs every other frame. Simulation speed is correct.

**The key insight**: `scene_tick` is always called with exactly `dt_sec = 1/60` seconds. Never more, never less. The simulation is deterministic regardless of render speed or hardware.

### The 100ms cap

```c
if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;
```

If the program was paused by a debugger, suspended with Ctrl-Z, or the computer went to sleep, `dt` could be millions of milliseconds. Without the cap, `sim_accum` would be enormous and the accumulator loop would fire thousands of physics ticks in a row — freezing the program for seconds catching up. The cap limits any single frame's contribution to 100ms. This means after a pause, at most 6 physics ticks fire (100ms / 16.7ms ≈ 6). The simulation "snaps" forward by a short amount rather than running in a long catch-up burst.

---

## 13. Render Interpolation — Smooth Motion Without Cheating

```c
float alpha = (float)sim_accum / (float)tick_ns;
```

After the accumulator loop, `sim_accum` holds the leftover time — how many nanoseconds into the *next* tick we are, but that tick has not fired yet.

If `tick_ns = 16,666,666 ns` and `sim_accum = 8,333,333 ns` (half a tick), then `alpha = 0.5`. We are halfway between the last physics tick and the next one.

For a ball moving at constant velocity, we can estimate where it *would be right now* by extrapolating:

```c
float draw_px = ball.px + ball.vx * alpha * dt_sec;
float draw_py = ball.py + ball.vy * alpha * dt_sec;
int cx = px_to_cell_x(draw_px);
int cy = px_to_cell_y(draw_py);
```

This draws the ball at its estimated current position, not its last simulated position.

### Why does this matter?

Without interpolation, a ball moving at 10 cells/second on a 60fps display would visually jump by 1/6 of a cell each physics tick. The ball appears to teleport in discrete steps rather than move smoothly. With interpolation, every render frame draws the ball at its exact sub-tick position — the motion appears perfectly smooth even though physics only runs 60 times per second.

**The technical term**: this is **forward extrapolation** for constant-velocity entities, and **linear interpolation (lerp)** for entities whose next-tick position is known:

```c
/* If you store both prev and current position: */
float draw_px = ball.prev_px + (ball.px - ball.prev_px) * alpha;
```

This lerp gives a position between the last two physics states, proportional to `alpha`. At alpha=0.0 you draw at the last physics position; at alpha=1.0 you draw at the next.

### Why alpha is always in [0.0, 1.0)

After the accumulator loop, `sim_accum < tick_ns` (the loop exits when accum drops below one tick). So `alpha = sim_accum / tick_ns < 1.0`. It is never 1.0 — that would mean exactly one tick's worth of time is left, and the loop would have fired one more time.

### When to ignore alpha

For entities that are not continuously moving — text, a static grid, a UI element — alpha is passed but ignored. `KeyGen` is an example: the characters are at fixed cell positions. There is no extrapolation to do. Alpha is accepted in the function signature for uniformity, cast to `(void)`, and that is fine.

---

## 14. Why Separate Simulation from Rendering?

This is one of the most important architectural decisions in the framework, and it is worth explaining explicitly.

### They run at different rates

Physics should run at a fixed rate for numerical stability and determinism. Rendering should run as fast as the display allows (typically 60fps), or can be capped to save CPU. These rates can be different and should be independent.

With separation:
- Press `]` to raise sim to 120Hz: more accurate physics, same 60fps display
- Press `[` to lower sim to 10Hz: visible "chunky" physics updates, educational
- The terminal is slow (40fps render): physics still runs at exactly 60Hz

Without separation, lowering the sim rate also lowers the render rate. Raising it makes both faster. They are coupled.

### They need different information

`scene_tick` needs: the time delta (`dt_sec`), the world boundaries (`cols, rows`). It does not know about ncurses, color pairs, or what character represents a ball.

`scene_draw` needs: the current state of all entities, the ncurses window, the terminal dimensions, and the interpolation factor (`alpha`). It does not know about time, physics equations, or numerical integration.

When you mix them, functions need both sets of information. They become large, have many parameters, are harder to test, and harder to reuse.

### Testing

Physics functions can be tested without a terminal. Call `scene_tick` 1000 times with fixed dt, check the final state. This is unit testing. You cannot do it if physics is interleaved with ncurses calls, because `initscr()` requires a terminal to be attached.

---

## 15. Two Delta Times — Not a Mistake

A common point of confusion: the code has *two* different delta time values. They look similar but are used for completely different things.

### Quick reference

| Variable | Type | Unit | Source | Used for |
|----------|------|------|--------|----------|
| `dt` | `int64_t` | nanoseconds | `clock_ns()` — real wall clock | filling `sim_accum`, fps counter, frame-cap sleep |
| `dt_sec` | `float` | seconds | `tick_ns / NS_PER_SEC` — constant derived from `sim_fps` | passed to `scene_tick` as the physics timestep |

---

### `dt` (int64_t, nanoseconds) — the real elapsed time

```c
int64_t now = clock_ns();
int64_t dt  = now - frame_time;
frame_time  = now;
if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;
```

This is the actual wall-clock time since the last frame. It is measured from `CLOCK_MONOTONIC` — a hardware counter that cannot go backwards and is unaffected by NTP adjustments or system clock changes. It varies every frame: sometimes 16 ms, sometimes 17 ms, sometimes 14 ms, depending on OS scheduling, background load, and the terminal emulator's own overhead.

It is used only for three things:
1. **Filling the accumulator:** `sim_accum += dt` — this is the only place real time enters the physics system
2. **The frame cap sleep:** `clock_sleep_ns(RENDER_NS - elapsed)` uses it to decide how long to sleep
3. **The fps counter:** `fps_accum += dt` accumulates real time between display updates

It is **never** passed to `scene_tick`. Handing raw wall-clock time to a physics integrator is the single most common mistake in simulation programming.

#### The 100 ms cap

```c
if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;
```

Without this cap, any pause in the process (debugger breakpoint, Ctrl-Z, laptop lid close, OS hiccup) would produce a `dt` of seconds or minutes. The accumulator would fill up and the `while` loop would fire hundreds or thousands of physics ticks in a row — the program appears frozen catching up. The cap limits the damage: at most `100ms / 16.7ms ≈ 6` catch-up ticks. The simulation snaps forward a small amount and continues normally.

---

### `dt_sec` (float, seconds) — the fixed physics timestep

```c
int64_t tick_ns = TICK_NS(app->sim_fps);          /* e.g. 16,666,666 ns at 60 Hz */
float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;   /* 0.016666... f           */
```

This is computed once from `sim_fps` — a setting, not a measurement. It does not depend on how long any frame actually took. If `sim_fps = 60`, then `dt_sec = 1/60 ≈ 0.01667 s` on every single call to `scene_tick`, no matter what. It is the same value for the first tick of the program and the millionth.

This fixed value is the timestep that every physics integrator in every simulation in this project uses:

```c
ball.vx += ax * dt_sec;
ball.px += ball.vx * dt_sec;
```

```c
/* RK4 in double_pendulum.c */
k1 = f(state, dt_sec);
k2 = f(state + k1 * 0.5f * dt_sec, dt_sec);
...
```

```c
/* Gauss-Seidel in navier_stokes.c */
diffuse(u, u0, visc, dt_sec, iter);
```

All of them receive `dt_sec`. None of them receive `dt`.

---

### Why is this distinction critical?

#### The naive (wrong) approach

A beginner might write:

```c
while (running) {
    int64_t now = clock_ns();
    float dt = (float)(now - last) / 1e9f;
    last = now;

    ball.x += ball.vx * dt;   /* variable timestep — looks right, isn't */
    draw();
}
```

This is called a **variable-timestep loop**. Physics advances by a different amount each frame. It seems correct — you scale by the actual elapsed time. It has four serious problems:

**1. Numerical instability**

Physics integrators (Euler, RK4, Verlet) are derived under the assumption that `dt` is small and constant. Most have a stability threshold — a maximum `dt` above which the approximation diverges. For a spring: `dt_max = 2 / sqrt(k/m)`. For explicit Euler on a stiff ODE, even a 2× spike in `dt` can cause the simulation to explode. The 100 ms cap helps but does not eliminate this: a frame that genuinely takes 33 ms (slow terminal, background load) gives `dt = 0.033` instead of `0.0167` — 2× larger, which can destabilise many simulations.

**2. Non-determinism**

Run the program twice and you get a different trajectory for every particle, pendulum, and fluid cell — because the sequence of `dt` values from the OS is never exactly the same. You cannot reproduce a bug. You cannot write a test that checks "after 100 steps the ball is at position X" because the answer depends on the timing of each frame.

**3. Speed variation across hardware**

A fast machine runs at 120 fps: `dt ≈ 0.0083`. A slow machine runs at 15 fps: `dt ≈ 0.067`. Both use `dt` directly, so both advance the physics by the correct total time over one second — but with very different step sizes. RK4 with `dt = 0.067` is far less accurate than with `dt = 0.0083`. The simulation looks visibly different on each machine.

**4. Floating-point collision skipping**

Two objects that would collide at the midpoint of a frame might completely pass through each other if `dt` is large enough that the discrete step skips the collision. Fixed `dt_sec` keeps the step small enough that collision geometry is reliable.

#### How the accumulator solves all four

```c
sim_accum += dt;                              /* real time enters here  */
while (sim_accum >= tick_ns) {
    scene_tick(&app->scene, dt_sec, cols, rows);  /* fixed dt_sec always */
    sim_accum -= tick_ns;
}
```

The accumulator acts as a buffer between the real clock and the physics. Real time accumulates in `sim_accum` in whatever irregular chunks the OS provides. Physics drains it in precise fixed scoops of `tick_ns`. The physics function never knows that a frame was slow or fast — it always sees `dt_sec = 1/60`.

- **Stability:** `dt_sec` is chosen to be safely below every integrator's stability threshold
- **Determinism:** same `dt_sec` every tick → same result every run on every machine
- **Speed independence:** physics advances at exactly `sim_fps` ticks per real second regardless of render rate
- **Collision reliability:** step size is predictable and bounded

#### The lie, and why it is necessary

The accumulator says: "I will call `scene_tick` exactly N times this frame, each time telling the physics that exactly `tick_ns` nanoseconds have passed."

That is a lie. The frame might have taken 33 ms, not 16.7 ms. But the lie is deliberate and beneficial. It gives physics a stable, deterministic, hardware-independent substrate. The real elapsed time is accounted for correctly at the accumulator level — two ticks fire for a 33 ms frame instead of one. The *total* simulated time is still correct; only the *per-tick* granularity is fixed.

`dt_sec` is not an approximation of reality. It is a contract: "this simulation integrates in steps of exactly 1/60 second."

---

### Where each variable appears in the code

| Location | Uses `dt` | Uses `dt_sec` |
|----------|-----------|----------------|
| Accumulator fill | `sim_accum += dt` | — |
| Frame cap sleep | `RENDER_NS - elapsed(dt)` | — |
| FPS display | `fps_accum += dt` | — |
| `scene_tick` call | — | passed as argument |
| Every physics integrator | — | `vx += ax * dt_sec` etc. |
| `alpha` computation | `sim_accum / tick_ns` (both) | — |

---

### The interpolation factor `alpha`

```c
float alpha = (float)sim_accum / (float)tick_ns;
```

After the accumulator loop, `sim_accum < tick_ns`. This leftover is how far into the *next* tick we are — a tick that has not fired yet. `alpha ∈ [0.0, 1.0)` represents a fraction of `dt_sec`.

Drawing entities at their last simulated position (alpha ignored) causes visible jitter: positions snap in discrete jumps of one physics step. Drawing at `lerp(prev_pos, curr_pos, alpha)` gives smooth sub-tick motion at any render rate, without the physics having to run more often.

`alpha` is derived from both time values — `sim_accum` was filled by real `dt`, and `tick_ns` is the fixed physics step — making it the bridge between the two worlds.

---

## 16. The ncurses Double Buffer

The ncurses double-buffer is not something you implement — it is built into ncurses and always active. Understanding it explains why the render sequence must follow a specific order.

### What a double buffer is

In graphics, a double buffer means: have two copies of the frame. Write the new frame into the back buffer while the front buffer is being displayed. When the new frame is ready, swap them. The display always sees a complete frame, never a half-rendered one.

### How ncurses implements it

ncurses maintains two virtual screens:

**`curscr`** — what ncurses believes is currently displayed on the physical terminal. Every character, color, and attribute for every cell.

**`newscr`** — the frame you are currently building. Every `mvwaddch`, `wprintw`, `wattron` call writes into `newscr`.

When you call `doupdate()`:
1. ncurses compares `newscr` with `curscr` cell by cell
2. For each cell that differs, it generates the minimal terminal escape sequence to update just that cell
3. It sends all those sequences in one write to the terminal file descriptor
4. It sets `curscr = newscr`

The result: only changed cells are transmitted. A frame where only 10 characters moved sends maybe 100 bytes to the terminal. A full redraw would send many thousands. This is why animations are smooth even on slow terminal connections.

### The correct frame sequence

```c
erase();              /* blank newscr */
scene_draw(…);        /* write scene into newscr */
mvprintw(…);          /* write HUD into newscr */
wnoutrefresh(stdscr); /* mark newscr ready */
doupdate();           /* diff + send + curscr = newscr */
```

**Why `erase()` first?**

Last frame, the scene might have had a ball at (10, 20). This frame it moved to (11, 21). Without `erase()`, cell (10, 20) still has the old ball character in `newscr`. The new frame would show both the old and new positions — a ghost trail.

`erase()` clears `newscr` to spaces. Then `scene_draw` writes the new positions. The diff between `newscr` (new position + cleared old) and `curscr` (old position) correctly erases the old character and draws the new one.

**Why is HUD drawn after scene_draw?**

ncurses has no concept of layers or z-order. Last write wins. If the scene writes something at row 0 (where the HUD lives), the HUD must overwrite it. Drawing the HUD last guarantees it is always visible regardless of what the scene does.

### The common mistake: a manual back/front window pair

Some beginners create two WINDOWs and blit one to the other:

```c
WINDOW *back  = newwin(LINES, COLS, 0, 0);
WINDOW *front = stdscr;
/* ... draw into back ... */
copywin(back, front, 0,0, 0,0, LINES-1, COLS-1, FALSE);
refresh();
```

This breaks ncurses' diff engine. ncurses tracks what is in `curscr`. When you `copywin` from `back` to `front`, you copy raw characters but ncurses still thinks `curscr` has the old values. The next `doupdate()` computes the diff against the wrong reference and sends too much (or the wrong) data. You get ghost trails, missing characters, and flickering.

**The rule**: use exactly one window (`stdscr`) for all drawing. Let ncurses manage the double buffer internally.

---

## 17. Signal Handling — Graceful Shutdown

```c
signal(SIGINT,   on_exit_signal);
signal(SIGTERM,  on_exit_signal);
signal(SIGWINCH, on_resize_signal);

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

atexit(cleanup);
```

### What are signals?

Signals are asynchronous notifications sent to a process by the OS or another process. They interrupt normal execution at any point — even in the middle of a function call.

**`SIGINT`** — sent when the user presses Ctrl+C. Default action: terminate the process.

**`SIGTERM`** — sent by `kill processid` or when the system shuts down. Default action: terminate.

**`SIGWINCH`** — sent when the terminal window is resized. Default action: ignored.

### What happens without signal handling?

**Without SIGINT/SIGTERM handling:** The default action terminates the process immediately, without calling `endwin()`. The terminal is left in raw mode — the shell prompt appears but echo is disabled, so typing is invisible. Input is not line-buffered, so arrow keys send raw escape sequences. The user has to type `reset` (blindly) to fix the terminal.

**Without SIGWINCH handling:** The terminal is resized but ncurses still thinks it is the old size. Drawing outside the new boundaries silently fails. Parts of the screen are blank or show corrupted characters.

### Why `atexit(cleanup)` in addition to `screen_free`?

`screen_free` is called at the end of `main()`. But what if the program calls `exit()` from somewhere else? What if `assert()` fails? What if a library calls `exit()` internally? `atexit` registers a function to be called in all of these cases. It is a safety net. Having both is not redundant — it is defence in depth.

---

## 18. App / Screen / Scene — Why Three Structs?

The three structs form a hierarchy:

```
App {
    Scene scene {
        KeyGen kg { slots[], timers[], ... }
    }
    Screen screen { cols, rows }
    int sim_fps
    volatile sig_atomic_t running
    volatile sig_atomic_t need_resize
}
```

### Why not one giant struct?

Putting everything in one struct blurs the boundaries between concerns. The signal handler would need to know about the physics entity. The physics function would live next to ncurses initialisation code. The struct would be hundreds of fields and growing.

Three structs = three clearly bounded domains:

**`Scene`** — "what is being simulated." Physics entities, their state, their rules. No ncurses.

**`Screen`** — "how it is displayed." Terminal dimensions, ncurses session. No physics.

**`App`** — "the overall program." Owns one of each. Owns the loop state (sim_fps, running, need_resize). Signal handlers live here. Input handling lives here.

### Why does App own both Scene and Screen by value (not pointer)?

```c
typedef struct {
    Scene  scene;    /* embedded, not Scene* */
    Screen screen;   /* embedded, not Screen* */
    ...
} App;
```

Embedding by value means:
1. One allocation. `g_app` is a single global variable. No `malloc`, no `free`, no risk of use-after-free.
2. Locality. All related data is adjacent in memory. CPU cache performance is better.
3. Lifetime. `scene` and `screen` live exactly as long as `app`. No lifetime management needed.

A pointer (`Scene *scene`) would require a separate allocation and a `free`. For a program with exactly one scene and one screen, that is unnecessary complexity.

### The `App *app = &g_app` pattern

```c
App *app = &g_app;
```

Inside `main`, the code uses `app->scene` and `app->screen` via a pointer. This makes it easy to switch to a heap-allocated `App` if needed (just change `App *app = &g_app` to `App *app = malloc(sizeof *app)`). It also reads more naturally than `g_app.scene.kg.paused` — `app->scene.kg.paused` shows the ownership hierarchy clearly.

---

## 19. The Frame Cap — Sleep Before Render

```c
int64_t elapsed = clock_ns() - frame_time + dt;
clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
```

### What this does

The target frame duration is `NS_PER_SEC / 60 = 16,666,666 ns`. `elapsed` is how many nanoseconds have been spent so far this frame (on resize, dt measurement, physics ticks). The sleep fills the remaining budget so that the frame takes exactly 16.7ms from start to render.

### Why sleep BEFORE render, not after?

The render call (`screen_draw` + `screen_present`) takes time — typically 0.5ms to 5ms depending on terminal speed and how many cells changed. If you sleep *after* render, that time is not accounted for in the next frame's dt measurement. The next frame's dt is shorter, the accumulator gets less, and physics runs slower.

More importantly: if you sleep *after* render, the sleep budget is:
```
sleep = frame_budget - (physics_time + render_time)
```

On a slow terminal where render takes 10ms and physics takes 2ms:
```
sleep = 16.7ms - 12ms = 4.7ms
```

That gives 60fps. But if the terminal slows down to 14ms:
```
sleep = 16.7ms - 16ms = 0.7ms
```

Still 60fps, barely. If it hits 18ms:
```
sleep = 16.7ms - 18ms = -1.3ms  → no sleep
```

The program now runs flat-out, wasting CPU on a slow terminal.

With sleep **before** render:
```
sleep = frame_budget - physics_time
```

Physics takes 2ms:
```
sleep = 16.7ms - 2ms = 14.7ms
```

You sleep 14.7ms, then render (takes 10ms). Total: 26.7ms → 37fps. The program responsibly throttles to what the terminal can handle rather than thrashing.

This is why `clock_sleep_ns` is called *before* `screen_draw` and `screen_present`.

---

## 20. Putting It All Together — The Full Data Flow

Here is the complete path from "user presses a key" to "pixels change on screen":

```
① OS sends SIGWINCH / SIGINT / SIGTERM
   → signal handler sets g_app.running = 0 or g_app.need_resize = 1
   → handled at top of next loop iteration

② clock_ns() measures real elapsed time dt (ns)
   dt capped at 100ms
   dt added to sim_accum

③ Fixed timestep loop:
   while (sim_accum >= tick_ns):
       scene_tick(scene, dt_sec, cols, rows)
           → keygen_tick(kg, dt_sec)
               → timers[i] -= scaled_dt
               → if timer <= 0: new random char, reset timer
       sim_accum -= tick_ns

④ alpha = sim_accum / tick_ns

⑤ fps_accum += dt
   if fps_accum >= 500ms: update fps_display

⑥ elapsed = now - frame_time + dt
   sleep(16.7ms - elapsed)   ← frame cap

⑦ screen_draw(screen, scene, fps, sim_fps, alpha, dt_sec)
       erase()
       scene_draw(scene, stdscr, cols, rows, alpha, dt_sec)
           → layout computations (centering)
           → wattron(COLOR_PAIR(…) | A_BOLD)
           → mvwaddch / mvwprintw for each slot
           → wattroff(…)
       mvprintw(HUD) last

   screen_present()
       wnoutrefresh(stdscr) → copies stdscr into newscr
       doupdate()           → diff newscr vs curscr → write changes → terminal

⑧ ch = getch()           ← non-blocking, returns ERR if no key
   app_handle_key(app, ch) → modifies scene/app state
```

---

## 21. How to Write Your Own Animation Using This Framework

Here is the minimal checklist for adding a new simulation to this project:

### Step 1: Define your constants in §1

```c
enum {
    MY_ENTITY_COUNT = 50,
    /* ... */
};
#define MY_SPEED 200.0f   /* pixels per second */
```

### Step 2: Write your physics struct and functions in §5

```c
typedef struct {
    float px, py;    /* position in pixel space */
    float vx, vy;    /* velocity in px/sec */
    int   color;
} MyEntity;

static void my_entity_init(MyEntity *e, int pw, int ph) {
    e->px = (float)(rand() % pw);
    e->py = (float)(rand() % ph);
    e->vx = MY_SPEED * (1.0f - 2.0f*(rand()%2));
    e->vy = MY_SPEED * (1.0f - 2.0f*(rand()%2));
    e->color = (rand() % N_COLORS) + 1;
}

static void my_entity_tick(MyEntity *e, float dt, int pw, int ph) {
    e->px += e->vx * dt;
    e->py += e->vy * dt;
    /* bounce off walls */
    if (e->px < 0)  { e->px = 0;        e->vx = -e->vx; }
    if (e->px > pw) { e->px = (float)pw; e->vx = -e->vx; }
    if (e->py < 0)  { e->py = 0;        e->vy = -e->vy; }
    if (e->py > ph) { e->py = (float)ph; e->vy = -e->vy; }
}
```

### Step 3: Add to Scene in §6

```c
typedef struct {
    MyEntity entities[MY_ENTITY_COUNT];
} Scene;

static void scene_init(Scene *s) {
    memset(s, 0, sizeof *s);
    /* need cols/rows for initial placement — use defaults */
    for (int i = 0; i < MY_ENTITY_COUNT; i++)
        my_entity_init(&s->entities[i], pw(80), ph(24));
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
    int world_w = pw(cols), world_h = ph(rows);
    for (int i = 0; i < MY_ENTITY_COUNT; i++)
        my_entity_tick(&s->entities[i], dt, world_w, world_h);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)rows;
    for (int i = 0; i < MY_ENTITY_COUNT; i++) {
        const MyEntity *e = &s->entities[i];
        /* extrapolate position using alpha */
        float draw_px = e->px + e->vx * alpha * dt_sec;
        float draw_py = e->py + e->vy * alpha * dt_sec;
        int cx = px_to_cell_x(draw_px);
        int cy = px_to_cell_y(draw_py);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(e->color) | A_BOLD);
        mvwaddch(w, cy, cx, '*');
        wattroff(w, COLOR_PAIR(e->color) | A_BOLD);
    }
}
```

### Step 4: Change nothing in §2, §3, §7, §8, main()

The main loop, signal handlers, ncurses setup, clock functions — all unchanged. The framework handles everything. You only wrote the physics (§5) and plugged it into the scene (§6).

This is the power of the architecture: 80% of the code is identical across all 170+ simulations. Only §5 and the simulation-specific parts of §6 differ.

---

## 22. Common Mistakes and What They Look Like

### Mistake 1: Calling ncurses from the physics function

**Code:** `mvwaddch(stdscr, cy, cx, '*')` inside `scene_tick`.
**Symptom:** Flickering, torn frames, partial updates visible.
**Fix:** Move all drawing to `scene_draw`. Physics functions return void and only modify structs.

### Mistake 2: Using `clear()` instead of `erase()`

**Code:** `clear();` instead of `erase();` in `screen_draw`.
**Symptom:** Every frame, the entire terminal flickers slightly and redraws completely.
**Fix:** Use `erase()`. It clears the virtual buffer; `doupdate()` still sends only changed cells.

### Mistake 3: Forgetting `typeahead(-1)`

**Code:** Omitting `typeahead(-1)` from `screen_init`.
**Symptom:** When typing quickly, frames are drawn with missing characters. Especially visible on fast-typing users or slow SSH connections.
**Fix:** Add `typeahead(-1)` after `keypad()` in `screen_init`.

### Mistake 4: Variable timestep passed to physics

**Code:** `scene_tick(s, (float)dt / 1e9f, cols, rows)` where `dt` is the real elapsed nanoseconds.
**Symptom:** Physics runs at different speeds on different machines. On fast machines the simulation is accurate; on slow machines (or when a frame is delayed) entities jump or explode.
**Fix:** Pass `dt_sec = (float)tick_ns / NS_PER_SEC` — the fixed tick duration, not the real elapsed time.

### Mistake 5: Not handling resize

**Code:** Omitting `SIGWINCH` handling.
**Symptom:** Making the terminal smaller causes ncurses to try drawing at invalid positions. Making it larger leaves the new area blank.
**Fix:** Register `on_resize_signal`, check `need_resize` at the start of each iteration, call `screen_resize`.

### Mistake 6: Signal-handler calling endwin or printf

**Code:** `void on_exit_signal(int sig) { endwin(); exit(0); }`
**Symptom:** Works on Linux today, undefined behaviour on some platforms, can deadlock if the signal fires while inside an ncurses function.
**Fix:** Set a flag only. Do cleanup in the main loop after the flag is checked.

### Mistake 7: Not using `static` on internal functions

**Code:** `int64_t clock_ns(void) { ... }` without `static`.
**Symptom:** "multiple definition of `clock_ns`" linker error when compiling multiple `.c` files.
**Fix:** All internal functions are `static`. They are private to the file.

### Mistake 8: Forgetting `(chtype)(unsigned char)` cast on `mvwaddch`

**Code:** `mvwaddch(w, y, x, my_char)` where `my_char` is a `char`.
**Symptom:** Characters with values >= 128 (extended ASCII) display as garbled symbols or cause ncurses errors. `char` is signed on most platforms; values > 127 are negative, and the sign extension corrupts the `chtype`.
**Fix:** Always cast: `mvwaddch(w, y, x, (chtype)(unsigned char)my_char)`.

---

## Conclusion

`framework.c` is 860 lines, but the core ideas are five:

1. **Fixed timestep accumulator** — physics always runs at the same speed, regardless of render rate or hardware. This makes simulations deterministic, stable, and comparable across machines.

2. **Render interpolation (alpha)** — drawing happens between physics ticks using extrapolation. Motion appears smooth at any frame rate.

3. **Separation of simulation and rendering** — physics functions never call ncurses. Draw functions never modify physics state. They communicate only through the shared `Scene` struct.

4. **ncurses double buffer via wnoutrefresh+doupdate** — only changed cells are sent to the terminal each frame. Animations run smoothly even over slow SSH connections.

5. **Layered architecture (§1–§8)** — each section has one responsibility. The main loop and signal handlers never change. Only §5 (the physics entity) and §6 (the scene) differ between the 170+ animations in this project.

Master these five ideas and you understand not just `framework.c`, but every simulation in this repository — and the architecture used in game engines, physics simulators, and real-time visualisation systems everywhere.

---

*Source file: `ncurses_basics/framework.c`*
*Companion reference: `physics/bounce_ball.c` — the motion-physics reference implementation*
*Build: `gcc -std=c11 -O2 -Wall -Wextra ncurses_basics/framework.c -o framework -lncurses -lm`*
