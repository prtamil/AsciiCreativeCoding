# Terminal Animation Framework — Architecture

Reference implementation: `basics/bounce_ball.c`

---

## Table of Contents

1. [Overview](#1-overview)
2. [Coordinate Spaces](#2-coordinate-spaces)
3. [Fixed Timestep with Accumulator](#3-fixed-timestep-with-accumulator)
4. [Render Interpolation (Alpha)](#4-render-interpolation-alpha)
5. [Frame Cap — Sleep Before Render](#5-frame-cap--sleep-before-render)
6. [ncurses Double Buffer — How It Actually Works](#6-ncurses-double-buffer--how-it-actually-works)
7. [ncurses Optimisations](#7-ncurses-optimisations)
8. [Section Breakdown](#8-section-breakdown)
   - [§1 config](#1-config)
   - [§2 clock](#2-clock)
   - [§3 color](#3-color)
   - [§4 coords](#4-coords)
   - [§5 physics (ball / pendulum / particle …)](#5-physics-ball--pendulum--particle-)
   - [§6 scene](#6-scene)
   - [§7 screen](#7-screen)
   - [§8 app](#8-app)
9. [Main Loop — Annotated Walk-through](#9-main-loop--annotated-walk-through)
10. [Signal Handling and Cleanup](#10-signal-handling-and-cleanup)
11. [Adding a New Animation](#11-adding-a-new-animation)
12. [Software Rasterizer — raster/*.c](#12-software-rasterizer--rasterc)
    - [Pipeline Overview](#pipeline-overview)
    - [ShaderProgram and the Split Uniform Fix](#shaderprogram-and-the-split-uniform-fix)
    - [Framebuffer](#framebuffer)
    - [Mesh and Tessellation](#mesh-and-tessellation)
    - [Vertex and Fragment Shaders](#vertex-and-fragment-shaders)
    - [Displacement (displace_raster.c)](#displacement-displace_rasterc)

---

## 1. Overview

Every animation in this project follows the same layered loop:

```
┌─────────────────────────────────────────────────────────┐
│                        main loop                        │
│                                                         │
│  ① measure dt (wall-clock elapsed since last frame)     │
│  ② drain sim accumulator → fixed-step physics ticks     │
│  ③ compute alpha (sub-tick render offset)               │
│  ④ sleep to cap output at 60 fps (BEFORE render)        │
│  ⑤ draw frame at interpolated position → stdscr         │
│  ⑥ doupdate() → one diff write to terminal              │
│  ⑦ poll input                                           │
└─────────────────────────────────────────────────────────┘
```

The design separates three concerns:

| Concern | Where | Rate |
|---|---|---|
| Physics simulation | `scene_tick()` | Fixed (e.g. 60 or 120 Hz) |
| Rendering | `screen_draw()` + `doupdate()` | Capped at 60 fps |
| Input | `getch()` | Every frame, non-blocking |

Physics and rendering are deliberately decoupled. The sim can run at 120 Hz for accuracy while the display stays at 60 fps, or the sim can run at 24 Hz for a stylised effect while still rendering smoothly.

---

## 2. Coordinate Spaces

**The root problem with naive terminal animation:**

Terminal cells are not square. A typical cell is roughly twice as tall as it is wide in physical pixels (e.g. 8 px wide × 16 px tall). If you store a ball's position directly in cell coordinates and move it by `dx = 1, dy = 1` per tick, it travels twice as far horizontally as vertically in physical pixels. Diagonal motion looks skewed. Circles become ellipses.

**The fix — two coordinate spaces, one conversion point:**

```
PIXEL SPACE          (physics lives here)
─────────────────────────────────────────
• Square grid. One unit ≈ one physical pixel.
• Width  = cols × CELL_W   (e.g. 200 cols × 8  = 1600 px)
• Height = rows × CELL_H   (e.g.  50 rows × 16 =  800 px)
• All positions, velocities, forces are in pixel units.
• Speed is isotropic — 1 px/s covers equal physical distance in X and Y.

CELL SPACE           (drawing happens here)
─────────────────────────────────────────
• Terminal columns and rows.
• cell_x = px_to_cell_x(pixel_x)
• cell_y = px_to_cell_y(pixel_y)
• Only scene_draw() ever calls px_to_cell_x/y.
• Physics code never sees cell coordinates.
```

```c
/* bounce_ball.c §4 */
#define CELL_W   8
#define CELL_H  16

static inline int pw(int cols) { return cols * CELL_W; }  /* pixel width  */
static inline int ph(int rows) { return rows * CELL_H; }  /* pixel height */

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}
```

**Why `floorf(px/CELL_W + 0.5f)` instead of `roundf` or truncation:**

- `roundf` uses "round half to even" (banker's rounding). When `px/CELL_W` is exactly `0.5` it can round to 0 on one call and 1 on the next depending on FPU state. A ball sitting on a cell boundary oscillates between two cells every frame — visible flicker.
- Truncation `(int)(px/CELL_W)` always rounds down. Creates asymmetric dwell time — staircase effect.
- `floorf(x + 0.5f)` is "round half up" — always deterministic, breaks ties in the same direction, symmetric dwell time with no oscillation.

**Simulations that don't need two spaces** (sand.c, fire.c, flowfield.c) work directly in cell coordinates because cells themselves are the physics grid. Those files omit §4 entirely.

---

## 3. Fixed Timestep with Accumulator

**Why fixed timestep:**

A variable timestep simulation (where `dt` passed to `tick()` equals whatever the wall clock measured) produces physically incorrect results. If a frame takes twice as long as usual, the tick receives `2×dt` and objects overshoot. Springs explode, collisions are missed, anything that depends on a maximum step size breaks. Floating-point integration errors also grow non-linearly with `dt`.

**The accumulator pattern:**

```c
/* bounce_ball.c §8 main loop */
int64_t tick_ns = TICK_NS(app->sim_fps);   /* e.g. 1/60 s = 16,666,666 ns */
float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

sim_accum += dt;                           /* dt = measured wall-clock ns   */
while (sim_accum >= tick_ns) {
    scene_tick(&app->scene, dt_sec, cols, rows);
    sim_accum -= tick_ns;
}
```

How it works:

1. `sim_accum` is a nanosecond bucket.
2. Every frame, the measured wall-clock `dt` is added to the bucket.
3. While the bucket holds enough for a full tick, one fixed-size physics step is consumed.
4. The remainder stays in the bucket for next frame.

```
Frame 1:  dt = 18 ms   sim_accum = 18 ms   → 1 tick (16.7 ms), leftover = 1.3 ms
Frame 2:  dt = 15 ms   sim_accum = 16.3 ms → 1 tick,           leftover = 0.3 ms  (dropped a tick vs naive)
Frame 3:  dt = 20 ms   sim_accum = 20.3 ms → 1 tick,           leftover = 3.6 ms
Frame 4:  dt =  5 ms   sim_accum =  8.6 ms → 0 ticks           leftover = 8.6 ms  (frame was fast, no tick)
```

Physics runs at exactly `sim_fps` steps per second on average, regardless of render frame rate. The simulation is deterministic, stable, and numerically identical on any machine.

**The dt cap:**

```c
if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;
```

If the process is paused (debugger, suspend, sleep) and then resumed, `dt` would be huge and the accumulator would drain thousands of ticks in one frame, causing an apparent physics jump. The cap clamps to 100 ms maximum — the simulation just "pauses" rather than catching up.

---

## 4. Render Interpolation (Alpha)

**The problem alpha solves:**

After draining the accumulator, `sim_accum` still holds the leftover nanoseconds — the time elapsed into the *next* tick that has not fired yet. If we draw objects at their last ticked position, we are drawing them up to one full tick behind wall-clock "now". At 60 Hz this is a 0–16 ms lag, visible as micro-stutter when the render frame lands just before a tick fires.

**The computation:**

```c
/* bounce_ball.c §8, immediately after the accumulator loop */
float alpha = (float)sim_accum / (float)tick_ns;
```

`alpha` ∈ [0.0, 1.0):

- `0.0` → render fires exactly on a tick boundary; draw position equals physics position.
- `0.9` → render fires 90% of the way through the next tick; draw position is projected 90% of a tick ahead.

**How it is used in `scene_draw`:**

```c
/* bounce_ball.c §6 scene_draw */
float draw_px = b->px + b->vx * alpha * dt_sec;
float draw_py = b->py + b->vy * alpha * dt_sec;
```

Each object's draw position is extrapolated forward by `alpha × dt_sec` seconds from its last ticked position using its current velocity. The drawn position tracks wall-clock "now" to within rendering error.

**Extrapolation vs true interpolation:**

This is technically *forward extrapolation* (predict from current state). True interpolation would store the previous tick's position and lerp between `prev` and `current`. For constant-velocity physics (bounce_ball.c), forward extrapolation is numerically identical to interpolation and requires no extra storage. For non-linear forces (spring_pendulum.c, fireworks.c), use proper lerp:

```c
/* spring_pendulum.c §6 */
float draw_r     = p->prev_r     + (p->r     - p->prev_r)     * alpha;
float draw_theta = p->prev_theta + (p->theta  - p->prev_theta) * alpha;
```

**When the scene is paused:**

`scene_tick` is skipped so physics positions do not change, but `alpha` still advances each frame. The draw position drifts slightly from the frozen physics position. This is imperceptible (less than one cell over the pause duration) and self-corrects when unpaused. To get pixel-perfect freeze, zero `alpha` when paused: `float alpha = scene.paused ? 0.0f : ...`

---

## 5. Frame Cap — Sleep Before Render

**The naive mistake:**

```c
/* WRONG ORDER */
screen_draw(...);          /* terminal I/O — unpredictable duration */
screen_present();          /* doupdate() — more terminal I/O        */
getch();                   /* input poll                            */

/* measure elapsed and sleep */
int64_t elapsed = clock_ns() - frame_time + dt;
clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
```

When the sleep is measured *after* terminal I/O, the elapsed time includes however long `doupdate()` and `getch()` took. On a slow terminal, a large frame might push `elapsed` past the budget entirely, making the sleep zero — the next frame starts immediately, the loop runs full-speed, and the frame rate becomes erratic.

**The correct order:**

```c
/* bounce_ball.c §8 — correct */

/* ① measure elapsed since frame_time (which is now = start of this frame) */
int64_t elapsed = clock_ns() - frame_time + dt;

/* ② sleep the remaining 60fps budget BEFORE any terminal I/O */
clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

/* ③ now do terminal I/O — the sleep has already consumed its budget */
screen_draw(...);
screen_present();
getch();
```

By sleeping first, the measurement captures only physics computation time (cheap, fast, predictable). The terminal I/O still takes variable time, but it is now "free" — it happens after the budget is spent, so it cannot cause the next frame to start late.

**`clock_sleep_ns` implementation:**

```c
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;          /* already over budget — don't sleep */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}
```

The `ns <= 0` guard handles frames where physics was unusually expensive — the sleep is simply skipped rather than causing undefined behaviour with a negative `nanosleep`.

---

## 6. ncurses Double Buffer — How It Actually Works

A common mistake in terminal animation is creating two `WINDOW*` objects and swapping them each frame ("front/back buffer"). This is wrong. ncurses already maintains an internal double buffer:

```
curscr   — what ncurses believes is currently on the physical terminal
newscr   — the frame you are building this render step
```

Every `mvwaddch`, `wattron`, `werase`, `mvprintw` call writes into `newscr`. Nothing reaches the terminal until you call `doupdate()`.

`doupdate()` computes `newscr − curscr` (the diff of changed cells only), sends that minimal set of escape codes to the terminal fd, then updates `curscr = newscr`. This is the double buffer. It is always present. It is not optional.

Adding a manual front/back `WINDOW` pair creates a *third* virtual screen that ncurses does not know about. When you copy from your back window into `stdscr` for display, the diff engine sees spurious changes on every cell, breaking its accuracy and producing ghost trails.

**The correct single-window model:**

```c
/* bounce_ball.c §7 */
erase();                          /* clear newscr — no terminal I/O   */
scene_draw(sc, stdscr, ...);      /* write scene into newscr           */
mvprintw(0, hud_x, "%s", buf);   /* write HUD into newscr (on top)    */
wnoutrefresh(stdscr);             /* copy stdscr into ncurses' newscr  */
doupdate();                       /* diff newscr vs curscr → terminal  */
```

**Properties:**

| Property | Result |
|---|---|
| No flicker | ncurses' diff engine never shows a partial frame |
| No ghost | `curscr` is always accurate — one source of truth |
| No tear | `doupdate()` is one atomic write to the terminal fd |
| HUD Z-order | Written last into same `stdscr` → always on top |

---

## 7. ncurses Optimisations

**`typeahead(-1)`**

```c
typeahead(-1);   /* in screen_init */
```

By default ncurses interrupts its output mid-flush to check whether there is input waiting on stdin. On fast terminals or when many cells change at once, this poll breaks up `doupdate()`'s write into multiple smaller writes, causing visible tearing. `typeahead(-1)` disables the check — ncurses writes the entire diff atomically.

**`nodelay(stdscr, TRUE)`**

```c
nodelay(stdscr, TRUE);
```

Makes `getch()` non-blocking. Without this, `getch()` blocks until a key is pressed, halting the entire loop. With `TRUE`, it returns `ERR` immediately when no key is available.

**`erase()` vs `clear()`**

- `clear()` marks every cell as changed and sends `\e[2J` (clear screen) to the terminal — a full repaint every frame, expensive and flickery.
- `erase()` clears ncurses' `newscr` internal buffer only. No terminal I/O. The diff engine will only send changes, not a full redraw.

Always use `erase()` in the render loop.

**`wnoutrefresh` + `doupdate` vs `wrefresh`**

- `wrefresh(w)` = `wnoutrefresh(w)` + `doupdate()` in one call.
- When you have only one window (`stdscr`), both are equivalent.
- The framework always uses `wnoutrefresh` + `doupdate` explicitly to make the two-phase pattern clear and to allow future multi-window compositing if needed.

**`curs_set(0)`**

Hides the terminal cursor. Without this, the cursor jumps to wherever the last `mvaddch` was called — visible as a flashing dot that moves around the screen every frame.

**`cbreak()` + `noecho()`**

- `cbreak()` delivers keystrokes immediately without waiting for Enter.
- `noecho()` prevents typed characters from being printed to the terminal.

---

## 8. Section Breakdown

Every C file in the project follows the same §-numbered section layout.

### §1 config

All tunable constants in one place. No magic numbers elsewhere in the file.

```c
enum {
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MIN      = 10,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,
    HUD_COLS         = 40,
    FPS_UPDATE_MS    = 500,
    BALLS_DEFAULT    =  5,
    BALLS_MAX        = 20,
    N_COLORS         =  7,
};

#define CELL_W   8
#define CELL_H  16
#define SPEED_MIN  300.0f
#define SPEED_MAX  600.0f
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))
```

`TICK_NS(fps)` converts a frame rate into a tick period in nanoseconds. Used in the accumulator and the frame cap.

### §2 clock

Two functions only: `clock_ns()` and `clock_sleep_ns()`.

```c
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
```

`CLOCK_MONOTONIC` never jumps backward (unlike `CLOCK_REALTIME` which can be adjusted by NTP or the user). Essential for a stable `dt` measurement.

### §3 color

Initialises ncurses color pairs. 256-color path with 8-color fallback.

```c
static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);   /* xterm-256 index */
        ...
    } else {
        init_pair(1, COLOR_RED, COLOR_BLACK);
        ...
    }
}
```

Color pairs are referenced later as `COLOR_PAIR(n)`. `A_BOLD` gives brighter foreground on both 256-color and 8-color terminals.

### §4 coords

Only present in animations that need isotropic physics (bounce_ball.c, spring_pendulum.c). Contains `pw()`, `ph()`, `px_to_cell_x()`, `px_to_cell_y()`. These are the *only* functions in the codebase that cross between pixel and cell space.

Simulations that work in cell coordinates (fire.c, sand.c, matrix_rain.c) do not have this section.

### §5 physics (ball / pendulum / particle …)

The core simulation object and its tick function. Has no knowledge of terminal dimensions, cell coordinates, or ncurses. Receives pixel boundaries from `scene_tick` if needed.

```c
/* bounce_ball.c §5 */
typedef struct {
    float px, py;   /* position in pixel space */
    float vx, vy;   /* velocity in pixel space */
    int   color;
    char  ch;
} Ball;

static void ball_tick(Ball *b, float dt, float max_px, float max_py)
{
    b->px += b->vx * dt;
    b->py += b->vy * dt;
    /* wall bounce */
    if (b->px < 0.0f)   { b->px = 0.0f;   b->vx = -b->vx; }
    if (b->px > max_px) { b->px = max_px;  b->vx = -b->vx; }
    if (b->py < 0.0f)   { b->py = 0.0f;   b->vy = -b->vy; }
    if (b->py > max_py) { b->py = max_py;  b->vy = -b->vy; }
}
```

### §6 scene

Owns the physics object(s) and exposes two functions: `scene_tick` and `scene_draw`.

**`scene_tick`** — advance physics one fixed step. Converts cell dimensions to pixel boundaries once (via `pw`/`ph`) then calls the object's tick function. No ncurses calls.

**`scene_draw`** — draw the current state into `stdscr`. Receives `alpha` and performs the render interpolation. This is the *only* function that calls `px_to_cell_x/y`. Nothing else in the program touches cell coordinates.

```c
/* bounce_ball.c §6 scene_draw (simplified) */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    for (int i = 0; i < s->n; i++) {
        const Ball *b = &s->balls[i];

        /* interpolated draw position — project forward by alpha ticks */
        float draw_px = b->px + b->vx * alpha * dt_sec;
        float draw_py = b->py + b->vy * alpha * dt_sec;

        /* clamp, convert to cell space */
        int cx = px_to_cell_x(draw_px);
        int cy = px_to_cell_y(draw_py);

        wattron(w, COLOR_PAIR(b->color) | A_BOLD);
        mvwaddch(w, cy, cx, (chtype)b->ch);
        wattroff(w, COLOR_PAIR(b->color) | A_BOLD);
    }
}
```

### §7 screen

Owns the ncurses session. Three responsibilities:

1. **`screen_init`** — `initscr`, set all ncurses options, `color_init`, query terminal size.
2. **`screen_draw`** — `erase()`, call `scene_draw`, write HUD. Builds the frame in `newscr`. No terminal I/O.
3. **`screen_present`** — `wnoutrefresh(stdscr)` + `doupdate()`. The single terminal write per frame.
4. **`screen_resize`** — `endwin()` + `refresh()` + re-query size. Called after `SIGWINCH`.

```c
/* bounce_ball.c §7 screen_draw */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  balls:%-2d  %s  spd:%d",
             fps, sc->n, sc->paused ? "PAUSED " : "running", sim_fps);
    int hud_x = s->cols - HUD_COLS;
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}
```

### §8 app

Owns `Scene`, `Screen`, `sim_fps`, and the signal flags. Entry point for everything.

**`App` struct:**

```c
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;
```

`running` and `need_resize` are `volatile sig_atomic_t` because they are written from signal handlers and read from the main loop. `volatile` prevents the compiler from caching the value in a register; `sig_atomic_t` guarantees the read/write is atomic.

**`app_handle_key`** — maps key codes to state changes (pause, add/remove objects, adjust speed). Returns `false` to signal quit.

**`app_do_resize`** — called when `need_resize` is set. Calls `screen_resize` to re-query terminal dimensions, then re-initialises or clamps the scene to fit the new size. Resets `frame_time` and `sim_accum` so the dt measurement does not include resize latency.

---

## 9. Main Loop — Annotated Walk-through

```c
int main(void)
{
    /* ── setup ───────────────────────────────────────────────────────── */
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);   /* Ctrl-C  → running = 0        */
    signal(SIGTERM,  on_exit_signal);   /* kill    → running = 0        */
    signal(SIGWINCH, on_resize_signal); /* resize  → need_resize = 1    */

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns(); /* start of current frame         */
    int64_t sim_accum   = 0;          /* leftover ns from previous ticks */
    int64_t fps_accum   = 0;          /* ns accumulated for FPS counter  */
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── ① resize check ──────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset so dt doesn't include resize */
            sim_accum  = 0;
        }

        /* ── ② dt measurement ────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;  /* wall-clock ns since last frame */
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;  /* cap at 100ms */

        /* ── ③ sim accumulator (fixed-step physics) ──────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ render interpolation alpha ────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;
        /* alpha ∈ [0, 1): how far into the next (unfired) tick we are */

        /* ── ⑤ FPS counter (reads dt, no I/O) ───────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap BEFORE terminal I/O ────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
        /*
         * elapsed = time spent on physics + FPS counter this frame.
         * We sleep whatever is left of the 1/60s budget.
         * By sleeping HERE (before doupdate), the measurement excludes
         * terminal I/O latency, so the cap is stable regardless of
         * how long the terminal write takes.
         */

        /* ── ⑦ draw + present (one doupdate flush) ───────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ input (non-blocking) ──────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
```

**Why `elapsed = clock_ns() - frame_time + dt`:**

`frame_time` was set to `now` at step ②. `clock_ns() - frame_time` measures only what happened since then (physics, FPS counter). Adding `dt` back gives total time elapsed since the *previous* frame's end — the full frame budget used. Subtracting from `NS_PER_SEC/60` gives the sleep needed to hit exactly 60 fps.

---

## 10. Signal Handling and Cleanup

```c
static App g_app;   /* global so signal handlers can reach it */

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }
```

**`atexit(cleanup)`** — registers `endwin()` to run when the process exits for any reason. This restores the terminal (re-enable echo, show cursor, etc.) even if the program crashes or is killed.

**`SIGWINCH`** — sent by the terminal emulator whenever the window is resized. The handler sets `need_resize = 1`. The main loop checks this flag at the top of each iteration and calls `app_do_resize`. The flag pattern avoids calling ncurses functions from inside the signal handler (which is async-signal-unsafe).

**`SIGINT` / `SIGTERM`** — set `running = 0`. The main loop exits cleanly on the next iteration, calls `screen_free` → `endwin`, and returns normally.

**`(void)sig`** — suppresses the `-Wunused-parameter` warning. Signal handlers must have the signature `void f(int)` but the value is not needed here.

---

## 11. Adding a New Animation

Follow this checklist to add a new file to the project:

1. **Copy the §1–§8 structure** from `bounce_ball.c` or the closest existing file.

2. **§1 config** — define your physics constants, `SIM_FPS_DEFAULT`, `HUD_COLS`, `NS_PER_SEC`, `NS_PER_MS`, `TICK_NS`.

3. **§4 coords** — include only if your physics needs isotropic (square-pixel) coordinates. If your simulation works directly in cell space (like a grid CA), omit it.

4. **§5 physics struct** — define your simulation object. Store all state in pixel (or cell) coordinates. No ncurses types here.

5. **Tick function** — accept `float dt` (seconds). Apply forces, integrate, handle boundaries. No ncurses calls.

6. **`scene_draw`** — receive `float alpha`. Compute interpolated draw positions. Call `px_to_cell_x/y` once per object. Use `mvwaddch` / `wattron` / `wattroff` to write into `stdscr`.

7. **`screen_draw`** — call `erase()`, then `scene_draw(…, alpha)`, then write the HUD. Return without flushing.

8. **`screen_present`** — `wnoutrefresh(stdscr)` + `doupdate()`. One call per frame.

9. **Main loop** — follow the exact order: resize → dt → accumulator → alpha → FPS counter → **sleep** → draw → present → input. Do not move the sleep.

10. **Build line** — `gcc -std=c11 -O2 -Wall -Wextra yourfile.c -o yourname -lncurses -lm`

---

---

## 12. Software Rasterizer — raster/*.c

The `raster/` folder contains four self-contained software rasterizers that render 3-D geometry into the ncurses terminal using ASCII characters. They share an identical pipeline; only the mesh and (for `displace_raster.c`) the shader set differ.

Files:

| File | Primitive | Notes |
|---|---|---|
| `torus_raster.c`    | UV torus  | 4 shaders, always-on back-face cull |
| `cube_raster.c`     | Unit cube | 4 shaders, toggleable cull, zoom |
| `sphere_raster.c`   | UV sphere | 4 shaders, toggleable cull, zoom |
| `displace_raster.c` | UV sphere | 4 displacement modes, 4 shaders, zoom |

### Pipeline Overview

```
tessellate_*()          — build Vertex/Triangle arrays once at init
    ↓
scene_tick()            — rotate model matrix, recompute MVP each frame
    ↓
pipeline_draw_mesh()    — for every triangle:
    vert shader              VSIn → VSOut   (model → clip space)
    clip reject              all 3 verts behind near plane → skip
    perspective divide       clip → NDC → screen cell coords
    back-face cull           2-D signed area ≤ 0 → skip
    bounding box             clamp to [0,cols-1] × [0,rows-1]
    rasterize                for each cell in bbox:
        barycentric test         outside triangle → skip
        z-interpolate            z-test against zbuf → skip if farther
        interpolate VSOut        world_pos, world_nrm, u, v, custom[4]
        frag shader              FSIn → FSOut
        luma → dither → cbuf
    ↓
fb_blit()               — cbuf → stdscr → doupdate
```

### ShaderProgram and the Split Uniform Fix

All four raster files define:

```c
typedef struct {
    VertShaderFn  vert;
    FragShaderFn  frag;
    const void   *vert_uni;   /* passed to vert() */
    const void   *frag_uni;   /* passed to frag() */
} ShaderProgram;
```

**Why two uniform pointers instead of one:**

The vertex and fragment shaders can require *different* uniform struct types. In `displace_raster.c`, `vert_displace` needs `DisplaceUniforms` (contains `disp_fn`, `time`, `amplitude`, `frequency`) while `frag_toon` needs `ToonUniforms` (contains `bands`). With a single `void *uniforms` pointer, one of the two shaders would receive a pointer to the wrong struct. When it casts and dereferences it — for example, calling `du->disp_fn(...)` where `disp_fn` is at a byte offset that lies inside `ToonUniforms.bands` — the result is a null or garbage function pointer and an immediate segfault.

The fix is a separate pointer per shader stage. The pipeline passes each pointer only to the shader that owns it:

```c
/* pipeline_draw_mesh — vertex stage */
sh->vert(&in, &vo[vi], sh->vert_uni);

/* pipeline_draw_mesh — fragment stage */
sh->frag(&fsin, &fsout, sh->frag_uni);
```

`scene_build_shader` sets both pointers appropriately for each shader combination:

| Active shader | `vert_uni`     | `frag_uni`        |
|---|---|---|
| phong         | `&s->uni`      | `&s->uni`         |
| toon          | `&s->uni`      | `&s->toon_uni`    |
| normals       | `&s->uni`      | `&s->uni`         |
| wireframe     | `&s->uni`      | `&s->uni`         |

For the toon case, `vert_uni = &s->uni` (the vertex shader only needs `Uniforms`) and `frag_uni = &s->toon_uni` (`frag_toon` needs `ToonUniforms.bands`). This is safe because `ToonUniforms` leads with `Uniforms base` as its first member, so `(const Uniforms *)vert_uni` is a valid alias — zero-offset rule.

This fix was applied to all four raster files even though only `displace_raster.c` strictly requires it, to prevent the same class of crash if shaders are ever extended.

### Framebuffer

```c
typedef struct { float *zbuf; Cell *cbuf; int cols, rows; } Framebuffer;
typedef struct { char ch; int color_pair; bool bold; } Cell;
```

- `zbuf[cols*rows]` — float depth buffer, initialised to `FLT_MAX` each frame
- `cbuf[cols*rows]` — output cell buffer, written to `stdscr` by `fb_blit()`
- `luma_to_cell(luma, px, py)` — Bayer 4×4 ordered dither maps `[0,1]` luminance to a Paul Bourke ASCII density character and one of 7 ncurses color pairs

### Mesh and Tessellation

```c
typedef struct { Vec3 pos; Vec3 normal; float u,v; } Vertex;
typedef struct { int v[3]; } Triangle;
typedef struct { Vertex *verts; int nvert; Triangle *tris; int ntri; } Mesh;
```

Each `tessellate_*()` function allocates and fills `Vertex` and `Triangle` arrays once at startup. The pipeline indexes into `mesh->verts` using `tri->v[vi]` — all indices are guaranteed in-bounds by the tessellation loop construction.

### Vertex and Fragment Shaders

Shaders are plain C functions accessed through function pointers:

```c
typedef void (*VertShaderFn)(const VSIn *in,  VSOut *out, const void *uni);
typedef void (*FragShaderFn)(const FSIn *in,  FSOut *out, const void *uni);
```

**Vertex shaders** — transform model-space position to clip space, output world-space position and normal for lighting:
- `vert_default` — standard MVP transform (torus/cube/sphere)
- `vert_normals` — same + packs world normal into `custom[0..2]`
- `vert_wire`    — same + pipeline injects barycentric coords into `custom[0..2]`
- `vert_displace` — displaces position along normal before transforming (displace only)

**Fragment shaders:**
- `frag_phong`   — Blinn-Phong + gamma correction
- `frag_toon`    — quantised diffuse (N bands) + hard specular threshold
- `frag_normals` — world normal → RGB (debug view)
- `frag_wire`    — `min(custom[0..2])` edge distance → discard interior, draw edge

**`custom[4]`** in `VSOut`/`FSIn` is a general-purpose interpolated payload. Each shader pair uses it differently:
- phong/toon — unused
- normals    — `custom[0..2]` = world normal components
- wireframe  — `custom[0..2]` = per-vertex barycentric identity vector `(1,0,0)/(0,1,0)/(0,0,1)`; after barycentric interpolation across the triangle, `min(custom[])` is the distance to the nearest edge

### Displacement (displace_raster.c)

Four displacement modes, each a pure function `float fn(Vec3 pos, float time, float amp, float freq)`:

| Mode   | Formula |
|---|---|
| RIPPLE | `sin(time + r*freq) * amp * taper`  — concentric rings from equator |
| WAVE   | `sin(time + x*f + y*f + z*f) * amp` — diagonal travelling wave |
| PULSE  | `breathe * amp * exp(-r*falloff)`   — whole sphere breathes |
| SPIKY  | `pow(|sin(x*f)*sin(y*f)*sin(z*f)|, 0.6) * amp` — spiky ball |

After displacing `pos += N * d`, the surface normal must be recomputed. The method is central difference:

```
d_t = displace(pos + eps*T) - displace(pos - eps*T)   ← finite diff along tangent T
d_b = displace(pos + eps*B) - displace(pos - eps*B)   ← finite diff along bitangent B
T'  = T*(2*eps) + N*d_t    ← displaced tangent vector
B'  = B*(2*eps) + N*d_b    ← displaced bitangent vector
N'  = normalize(cross(T', B'))
```

`DisplaceUniforms` extends `Uniforms` with `disp_fn`, `time`, `amplitude`, `frequency`, `mode`. It leads with `Uniforms base` so `&disp_uni` casts cleanly to `const Uniforms *` inside fragment shaders that only need the base lighting fields.

---

*This document describes the state of the framework as implemented across all C files in this repository. The canonical reference for any ambiguity is `basics/bounce_ball.c`.*
