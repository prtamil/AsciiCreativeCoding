/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * framework.c — Terminal Animation Framework: Complete Reference Template
 *
 * DEMO: Key Generator — 26 character slots centered on screen, each
 *       independently cycling through random printable ASCII characters
 *       (movie-style cryptographic key generation effect).
 *
 * This file is the canonical framework template for every animation in
 * this project. Study bounce_ball.c alongside this file — that is the
 * motion-physics reference; this is the stationary-entity reference.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config   — every tunable constant in one place
 *   §2  clock    — monotonic nanosecond clock + sleep
 *   §3  color    — ncurses color pair setup (256-color / 8-color fallback)
 *   §4  coords   — pixel↔cell conversion; the aspect-ratio fix
 *   §5  entity   — per-animation state and update logic  (KeyGen here)
 *   §6  scene    — entity pool; tick (fixed-step); draw (interpolated)
 *   §7  screen   — ncurses double-buffer display layer
 *   §8  app      — signals, resize, main loop
 * ─────────────────────────────────────────────────────────────────────
 *
 * Main loop order (same in every animation):
 *
 *   ① measure dt (wall-clock elapsed since last frame)
 *   ② drain sim accumulator → fixed-step physics ticks
 *   ③ compute alpha (sub-tick render offset ∈ [0,1))
 *   ④ sleep to cap at 60 fps  ← BEFORE render, not after
 *   ⑤ build frame in newscr  → erase → scene_draw → HUD
 *   ⑥ doupdate()             → one diff write to terminal
 *   ⑦ poll input (non-blocking getch)
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          re-randomise all slots
 *   + / =      faster character cycling
 *   -          slower character cycling
 *   ] / [      raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra framework.c -o framework -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Reference framework template demonstrating the canonical
 *                  pattern used by all animations in this project:
 *                  fixed-step physics accumulator + render interpolation.
 *
 * Data-structure : Fixed-step accumulator: sim_accum += dt each frame;
 *                  drain in SIM_TICK_NS steps: while (accum ≥ tick) {
 *                  sim_tick(); accum -= tick; }  This decouples physics
 *                  rate from render rate — physics always runs at the same
 *                  speed regardless of CPU or render load.
 *
 * Rendering      : Sub-tick interpolation: alpha = sim_accum/tick_ns ∈ [0,1).
 *                  Entity draw positions lerp between prev and current
 *                  simulated positions at alpha, giving smooth motion at
 *                  any render rate without modifying physics.
 *
 * Performance    : ncurses double-buffer: erase → draw → wnoutrefresh →
 *                  doupdate().  doupdate() sends only changed cells to the
 *                  terminal (diff), minimising write latency and flicker.
 *                  Render capped at TARGET_FPS using CLOCK_MONOTONIC sleep.
 *
 * References     :
 *   Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com) —
 *     foundational article on fixed-step + alpha-lerp interpolation.
 *     Every animation in this project follows this pattern.
 *   ncurses(3X) man pages — getch, doupdate, wnoutrefresh,
 *     init_pair, KEY_RESIZE.
 *   project CLAUDE.md — coding-style + framework-structure rules.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every animation in this project follows ONE recipe.  The recipe
 * is not an abstraction (no header, no library) — it's a CODE
 * SHAPE, copied and customised per file.  This file is the
 * canonical recipe in template form.  Every other animation file
 * has the same §1..§8 structure with §5 (entity) replaced by the
 * specific simulation.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture every animation file as a CAKE: the same 8 layers,
 * just different filling in §5.  Bottom layer (§1 config) is the
 * pantry; top layer (§8 app) is the icing.  The recipe (sections
 * + main loop order) is fixed; the cake's flavour comes from the
 * §5 entity logic.  This file's flavour is "26 slots cycling
 * random characters."  bounce_ball.c's is "two balls bouncing."
 * fluid/navier_stokes.c's is "Stam stable fluid."  Same cake,
 * different fillings.
 *
 * THE EIGHT SECTIONS
 * ──────────────────
 *
 *   §1 config    Every magic number lives here.  Tunable constants,
 *                feature flags, color pair IDs, sim/render rates.
 *                Code below NEVER hard-codes a literal — it reads
 *                from §1.
 *
 *   §2 clock     Monotonic timer + nanosecond sleep.  Foundation
 *                for both physics dt measurement and render-rate
 *                capping.  CLOCK_MONOTONIC unaffected by NTP /
 *                wall-clock changes.
 *
 *   §3 color     ncurses color pair initialisation.  256-color
 *                preferred; 8-color fallback for legacy terminals.
 *                init_pair() once, attron(COLOR_PAIR(n)) per use.
 *
 *   §4 coords    Pixel↔cell conversion + aspect-ratio fix.
 *                Terminal cells are ~2:1 tall; coords from physics
 *                live in PIXEL space (square units), then convert
 *                at the LAST POSSIBLE MOMENT before drawing.  Cell-
 *                space sims (CAs, sand, fire) skip this section.
 *
 *   §5 entity    The actual simulation.  Per-animation state +
 *                update logic.  This is the ONE section that
 *                differs across files.  KeyGen here; in
 *                bounce_ball.c it's Ball; in flocking.c it's
 *                Boid; in raster/cube_raster.c it's Mesh.
 *
 *   §6 scene     Entity pool + per-frame orchestration.  Calls
 *                §5 entity_tick() under fixed-step accumulator;
 *                calls §5 entity_draw() with sub-tick alpha.
 *
 *   §7 screen    ncurses init/cleanup + frame-present helpers
 *                (erase → wnoutrefresh → doupdate).
 *
 *   §8 app       Top-level: signal handling (SIGINT/SIGTERM/
 *                SIGWINCH), main loop, input dispatch.
 *
 * MAIN LOOP — IDENTICAL EVERY FILE
 * ────────────────────────────────
 *
 *   while (running):
 *     1. dt = clock_now - prev_clock;  prev_clock = clock_now
 *     2. handle resize if SIGWINCH fired
 *     3. sim_accum += dt
 *        while sim_accum >= SIM_TICK_NS:
 *          scene_tick()              ← fixed-step physics
 *          sim_accum -= SIM_TICK_NS
 *     4. alpha = sim_accum / SIM_TICK_NS    ← ∈ [0, 1)
 *     5. sleep_until_target_fps             ← sleep BEFORE render
 *     6. erase()
 *        scene_draw(alpha)                  ← interpolated render
 *        HUD draw (yellow + cyan, A_BOLD)
 *        wnoutrefresh + doupdate            ← one diff write
 *     7. handle input (non-blocking getch)
 *
 * KEY FORMULAS
 * ────────────
 *   Fixed-step accumulator:
 *     SIM_TICK_NS = NS_PER_SEC / sim_fps    (e.g. 1/60 sec)
 *     while accum >= SIM_TICK_NS:
 *       tick(); accum -= SIM_TICK_NS
 *
 *   Alpha (sub-tick fraction):
 *     alpha = accum / SIM_TICK_NS    ∈ [0, 1)
 *
 *   Render interpolation (constant velocity):
 *     draw_pos = pos + vel · alpha · dt
 *
 *   Render interpolation (general):
 *     draw_pos = lerp(prev_pos, cur_pos, alpha)
 *
 *   Pixel→cell:
 *     col = round(px / CELL_W)        (CELL_W typically 8)
 *     row = round(py / CELL_H)        (CELL_H typically 16)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • SPIRAL OF DEATH: if dt becomes huge (system suspend, lid
 *     close), the inner while loop fires hundreds of ticks
 *     trying to catch up.  Guard with `dt = min(dt, 100ms)`.
 *   • RESIZE: SIGWINCH sets a sig_atomic_t flag; main loop
 *     reads it before scene_draw and re-derives geometry.
 *     Never call ncurses functions FROM the signal handler.
 *   • SLEEP BEFORE RENDER: sleeping AFTER render means the
 *     diff write to terminal happens at variable time, causing
 *     visible jitter.  Sleep first, then write.
 *   • TYPEAHEAD: by default ncurses interrupts diff writes to
 *     peek at stdin; call typeahead(-1) to disable.
 *   • DOUBLE BUFFER: erase() + scene_draw + wnoutrefresh +
 *     doupdate gives ONE atomic diff per frame.  Naïve refresh()
 *     after each glyph causes flicker and high CPU.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Default config: 26 character slots cycle; HUD shows live
 *     fps stable around 60.
 *   • Press [/]: sim Hz changes; physics rate scales but render
 *     stays smooth.
 *   • Press +/-: per-slot character rate changes; visible.
 *   • Resize terminal: layout re-centres; no crash, no
 *     corruption.
 *   • Pause (space): all slots freeze; resume picks up
 *     identically.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that
 *      order as prose.  This file is the PROJECT'S TEMPLATE.
 *      Every other simulation in this project follows the same
 *      §1..§8 structure with the §5 entity replaced.
 *   2. §1 config — every constant.  Read first; it's the
 *      table of contents for the rest.
 *   3. §6 scene + §8 app — the MAIN LOOP.  Read AFTER tutorials
 *      T1-T4 below.  Any quirk you see in another file's main
 *      loop probably has its rationale in T2-T3 here.
 *   4. §5 entity — the SIMULATION-SPECIFIC code.  In this file
 *      it's KeyGen (slots cycling random characters).  In
 *      bounce_ball.c it's Ball physics.  In flocking.c it's
 *      Boid steering.  Same shape, different content.
 *   5. §2-§4, §7 — clock + colour + coords + screen.
 *      Self-contained utilities; read as needed.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   sim_accum       fixed-step accumulator (ns).  Drained one
 *                   SIM_TICK_NS at a time.
 *   alpha           sub-tick render offset ∈ [0, 1).
 *   prev_*, cur_*   per-entity snapshots for alpha-lerp draw.
 *   SIM_FPS         simulation tick rate.
 *   TARGET_FPS      render frame cap.
 *   dt              wall-clock seconds since last frame.
 *   g_running       sig_atomic_t flag — set false on SIGINT/TERM.
 *   g_resize        sig_atomic_t flag — set true on SIGWINCH.
 *
 * Background you need
 * ───────────────────
 *   - C11 + ncurses basics (initscr, mvaddch, init_pair).
 *   - The IDEA of frames-per-second + fixed-step physics.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - SDL / OpenGL / GPU rendering.  Pure terminal output.
 *   - Game engines (Unity, Godot).  This is the manual version.
 *   - Real-time scheduling.  Soft real-time at 60 fps is the
 *     budget; CLOCK_MONOTONIC + nanosleep is enough.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials that build the framework from first principles.
 * After reading these, you can write a new animation by COPYING
 * this file, replacing §5 entity, and tweaking §1 config.
 *
 *   T1  Why a "framework"? — the separation of concerns
 *   T2  The §1 config principle — no scattered magic numbers
 *   T3  Fixed-step physics — decoupling sim rate from render rate
 *   T4  Sub-tick alpha lerp — smooth render at any frame rate
 *   T5  ncurses double-buffering — one diff write per frame
 *   T6  Variable timestep — when fixed-step is overkill
 *   T7  Resize + signals — SIGWINCH and atexit cleanup
 *   T8  When to deviate — pixel-space vs cell-space, rotation, etc.
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHY A "FRAMEWORK"? — THE SEPARATION OF CONCERNS
 * ───────────────────────────────────────────────────
 * A naïve animation file mixes EVERYTHING in main():
 *
 *     int main() {
 *       initscr();
 *       while (running) {
 *         clear();
 *         // physics + drawing + input + colour + ...
 *         // 600 lines of imperative spaghetti
 *         refresh();
 *         usleep(16000);
 *       }
 *     }
 *
 * That works for 100 lines.  It DOES NOT scale to 1000 lines
 * with multiple entity types, fixed-step physics, alpha
 * interpolation, resize handling, signal cleanup, themes.
 *
 * The §1..§8 structure SEPARATES CONCERNS:
 *
 *   - Tunables in ONE place (§1)     ← fast iteration
 *   - Time / clock isolated (§2)     ← cross-platform
 *   - Colour setup once (§3)         ← consistent palette
 *   - Pixel/cell math isolated (§4)  ← aspect bugs caught here
 *   - Simulation logic in §5         ← THE WORK
 *   - Frame orchestration §6         ← clear loop pattern
 *   - I/O at the boundaries §7-§8    ← swap renderer easily
 *
 * Each section has ONE responsibility.  Reading any animation
 * file becomes "find §5 to see the simulation; everything else
 * is shared infrastructure I already understand."
 *
 * Compare across the project:
 *   - flocking/flocking.c §5 = Boid + steering forces
 *   - physics/cloth.c §5 = Particle + spring constraints
 *   - raster/cube_raster.c §5 = Mesh + vertex shader
 *   - matrix_rain/matrix_rain.c §5 = Column + cycling glyphs
 *
 * Same skeleton; diverse §5 logic.
 *
 * T2  THE §1 CONFIG PRINCIPLE — NO SCATTERED MAGIC NUMBERS
 * ────────────────────────────────────────────────────────
 * Every constant lives in §1.  Every literal in code below
 * §1 is a NAME from §1 (not a number).
 *
 * Wrong:
 *
 *     for (int i = 0; i < 100; i++) {     // 100 ← magic
 *       ...
 *       if (x > 1024) {                   // 1024 ← magic
 *         ...
 *
 * Right:
 *
 *     // §1 config:
 *     #define N_PARTICLES  100
 *     #define WORLD_W      1024
 *
 *     // ...
 *     for (int i = 0; i < N_PARTICLES; i++) {
 *       ...
 *       if (x > WORLD_W) {
 *         ...
 *
 * Why?
 *   - TUNING: change one constant in §1, behaviour changes
 *     globally.  No grep-and-replace, no missed sites.
 *   - DOCUMENTATION: the constant's NAME explains what the
 *     number means.  100 is opaque; N_PARTICLES is obvious.
 *   - TESTING: you can find every parameter at a glance.
 *
 * Project rule: any literal that ISN'T 0, 1, 2, or a tight
 * arithmetic identity (e.g. `i + 1`, `n / 2`) goes in §1.
 *
 * T3  FIXED-STEP PHYSICS — DECOUPLING SIM RATE FROM RENDER RATE
 * ─────────────────────────────────────────────────────────────
 * Naïve approach: simulation tick = render frame:
 *
 *     while (running):
 *       physics_step(dt)        ← uses variable dt
 *       render()
 *       sleep(target_dt)
 *
 * Problems:
 *   - Slow CPU → big dt → physics steps too far → numerical
 *     instability (springs explode, balls tunnel through walls).
 *   - Fast CPU → tiny dt → physics integration error
 *     accumulates.
 *   - Pause + resume → giant dt → "spiral of death" trying to
 *     catch up.
 *   - Physics behaviour DEPENDS on framerate — bad for
 *     reproducibility.
 *
 * Fixed-step accumulator (Glenn Fiedler 2004):
 *
 *     SIM_TICK_NS = 1/60 sec       (constant)
 *     sim_accum += dt              (per frame)
 *     while sim_accum >= SIM_TICK_NS:
 *       physics_step(SIM_TICK_NS)  ← always SAME dt
 *       sim_accum -= SIM_TICK_NS
 *
 * Properties:
 *   - Physics ALWAYS runs at exactly 60 Hz regardless of
 *     render fps.
 *   - One slow render frame fires multiple physics ticks to
 *     catch up.
 *   - One fast render frame fires zero ticks (just renders
 *     the same state again — but with alpha to interpolate;
 *     T4).
 *   - Reproducible: same input + same seed = same result.
 *
 * Spiral-of-death guard: cap dt at 100ms.  If you suspend
 * for an hour, on resume dt = 1ms (cap) instead of 1hr —
 * one tick fires, sim moves on; nothing tries to catch up
 * the missed hour.
 *
 * T4  SUB-TICK ALPHA LERP — SMOOTH RENDER AT ANY FRAME RATE
 * ─────────────────────────────────────────────────────────
 * Fixed-step physics produces a STAIRCASE of states.  At
 * 60 Hz physics, the world updates exactly 60 times/sec.
 * If you render at 120 Hz, every other frame shows the
 * SAME state — visible STUTTER.
 *
 * Fix: SUB-TICK INTERPOLATION.  Compute how far between two
 * physics steps the render fell:
 *
 *     alpha = sim_accum / SIM_TICK_NS    ∈ [0, 1)
 *
 * Render entities INTERPOLATED between their previous and
 * current physics state:
 *
 *     draw_pos = lerp(prev_pos, cur_pos, alpha)
 *
 * For constant-velocity entities, equivalent shortcut:
 *
 *     draw_pos = pos + vel · alpha · dt
 *
 * The renderer SEES smooth motion at the render's frame rate
 * (60, 120, 144Hz, whatever).  The physics still runs at its
 * fixed 60Hz.  Best of both worlds.
 *
 * Cost: each entity stores both PREV and CUR position.  Tiny.
 *
 * Watch the demo: press [ to drop sim Hz to 10 (very visible
 * stair-steps if rendering raw); the alpha lerp keeps the
 * KeyGen's slot transitions smooth.
 *
 * T5  NCURSES DOUBLE-BUFFERING — ONE DIFF WRITE PER FRAME
 * ───────────────────────────────────────────────────────
 * Naïve ncurses programs call refresh() after every cell
 * write:
 *
 *     mvaddch(0, 0, 'A'); refresh();
 *     mvaddch(0, 1, 'B'); refresh();
 *     mvaddch(0, 2, 'C'); refresh();
 *
 * That's 3 terminal writes for 3 characters.  At 1000
 * characters per frame and 60 fps = 180,000 writes/sec.
 * Visible flicker, high CPU, lag.
 *
 * Correct ncurses pattern:
 *
 *     erase();                          ← clear scratch buffer
 *     // many mvaddch / mvprintw calls  ← writes go to scratch
 *     wnoutrefresh(stdscr);             ← mark scratch dirty
 *     doupdate();                       ← ONE diff write
 *
 * `erase()` clears ncurses' INTERNAL SCRATCH BUFFER (newscr),
 * not the terminal.  All draw calls write to scratch.
 * `doupdate()` computes the DIFF between scratch and the
 * known terminal state, sends ONLY changed cells.
 *
 * Result: 1 write per frame regardless of how many cells
 * changed.  60 writes/sec total.
 *
 * Additional must-haves:
 *   - typeahead(-1)        ← prevents ncurses from
 *                            interrupting writes to peek stdin
 *   - nodelay(stdscr, true)← getch returns ERR if no input,
 *                            doesn't block
 *   - keypad(stdscr, true) ← arrow keys etc. → KEY_LEFT etc.
 *
 * T6  VARIABLE TIMESTEP — WHEN FIXED-STEP IS OVERKILL
 * ───────────────────────────────────────────────────
 * Fixed-step is REQUIRED for stiff physics (springs, fluids,
 * cloth).  But it's OVERKILL for stateless animations.
 *
 * STATELESS animation example: a clock face that draws hands
 * at the current wall-clock time.  No accumulating state,
 * no dt-dependent integration.  The clock at time t is a
 * pure function of t.  Just compute and draw.  No
 * accumulator needed.
 *
 * VARIABLE-TIMESTEP pattern (used in this project's
 * artistic/ folder + other stateless demos):
 *
 *     while (running):
 *       dt = clock_now - prev
 *       global_time += dt
 *       erase()
 *       draw_at_time(global_time)        ← pure function of t
 *       wnoutrefresh + doupdate
 *       sleep_to_target_fps
 *
 * Indicator that variable timestep is OK:
 *   - Sim has NO ACCUMULATING STATE that depends on dt
 *     (or the only state is a simple advancing clock).
 *   - No springs, no fluids, no contact dynamics, no
 *     numerical integration.
 *   - Want to speed up / slow down with time-scale slider.
 *
 * Indicator that fixed-step is REQUIRED:
 *   - Physics integration (Verlet, RK4, Euler) on stiff
 *     systems.
 *   - Constraint solvers (rigid body, cloth, ragdoll).
 *   - Reproducibility critical (same seed = same result).
 *
 * Most files in this project use FIXED-STEP for safety.
 * The artistic/ folder + a few others (matrix_rain,
 * sun_solar, pure visualisations) use variable.
 *
 * T7  RESIZE + SIGNALS — SIGWINCH AND atexit CLEANUP
 * ──────────────────────────────────────────────────
 * Three signals matter:
 *
 *   SIGINT (Ctrl-C) + SIGTERM    user wants to quit
 *   SIGWINCH                      terminal resized
 *
 * Signal handlers can run AT ANY MOMENT, including in the
 * middle of an mvaddch().  They MUST NOT call ncurses
 * functions, malloc, or anything not async-signal-safe.
 *
 * Standard pattern:
 *
 *     volatile sig_atomic_t g_running = 1;
 *     volatile sig_atomic_t g_resize = 0;
 *
 *     void on_signal(int sig) {
 *       if (sig == SIGWINCH) g_resize = 1;
 *       else                 g_running = 0;
 *     }
 *
 *     // in main:
 *     signal(SIGINT, on_signal);
 *     signal(SIGTERM, on_signal);
 *     signal(SIGWINCH, on_signal);
 *
 *     while (g_running) {
 *       if (g_resize) {
 *         g_resize = 0;
 *         endwin();          ← reset ncurses to new size
 *         refresh();         ← restart with new LINES/COLS
 *         re_derive_layout();
 *       }
 *       // ... rest of frame
 *     }
 *
 * Cleanup via atexit:
 *
 *     atexit(endwin);
 *
 * Ensures the terminal is restored even on abnormal exit
 * (assert fail, segfault if registered before, etc.).
 *
 * Without atexit endwin, a crash leaves the terminal in raw
 * mode — no echo, no line buffering, often garbled.  User
 * has to run `reset` or close the terminal.
 *
 * T8  WHEN TO DEVIATE — PIXEL-SPACE VS CELL-SPACE, ROTATION, etc.
 * ───────────────────────────────────────────────────────────────
 * The §1..§8 template fits MOST animations.  Variations:
 *
 *   CELL-SPACE simulations (CAs, fire, sand, matrix_rain):
 *     skip §4 coords entirely.  Each cell IS the simulation
 *     unit; no pixel↔cell math needed.
 *
 *   POLAR-COORDINATE simulations (mandalas, pulsars):
 *     §4 has polar→cell conversion with ASPECT correction
 *     (sin·0.5 to compensate for 2:1 cell aspect).
 *
 *   3-D RENDERING (raster/, raymarcher/, raytracing/):
 *     §4 expands into projection matrices, MVP, NDC mapping.
 *     §5 entity becomes Mesh + shaders.
 *
 *   STATELESS animations:
 *     replace fixed-step accumulator with variable dt;
 *     entity_tick becomes entity_at_time(t).
 *
 *   MULTI-LAYERED scenes:
 *     §6 scene becomes a list of LAYERS, each with its own
 *     entity pool; draw in painter's order.
 *
 * Common to ALL variations:
 *   - §1 config = magic numbers
 *   - §2 clock = monotonic time
 *   - §3 color = palette setup
 *   - §7 screen = ncurses init/cleanup
 *   - §8 app = signals + main loop
 *
 * The skeleton STAYS.  The flesh changes.
 *
 * For a NEW animation: copy this file, replace §5 entity
 * with your simulation, tweak §1 config, done.  No header
 * to include, no library to link beyond ncurses + libm.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here. Never scatter literals through the code.
 * Change behaviour by editing this block only.
 */
enum {
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    HUD_COLS         =  48,   /* max width of the HUD status string       */
    FPS_UPDATE_MS    = 500,   /* recalculate displayed fps every 500 ms   */

    N_COLORS         =   7,   /* number of color pairs defined in §3      */

    KEY_LEN          =  26,   /* character slots in the key display       */
};

/*
 * Printable ASCII range used for the cycling characters.
 * 0x21 '!' → 0x7E '~'  =  94 printable characters (space excluded).
 */
#define ASCII_FIRST  0x21
#define ASCII_LAST   0x7E
#define ASCII_RANGE  (ASCII_LAST - ASCII_FIRST + 1)   /* 94 */

/*
 * How fast each slot cycles: changes per second.
 * Each slot gets its own rate in [RATE_MIN, RATE_MAX] so the display
 * looks organic — not all chars flipping at the same beat.
 */
#define RATE_MIN   4.0f    /* slowest slot — 4  changes/sec  */
#define RATE_MAX  28.0f    /* fastest slot — 28 changes/sec  */

/*
 * Timing primitives.
 * TICK_NS(f) converts a frame rate (Hz) to nanoseconds per tick.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC never goes backwards (unlike CLOCK_REALTIME which can
 * jump on NTP adjustments).  Subtracting two consecutive clock_ns()
 * calls gives the true elapsed time regardless of system load.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for exactly ns nanoseconds.
 *
 * Called BEFORE render (see §8) so that the sleep budget covers only
 * physics time — not terminal I/O.  If ns ≤ 0 the frame is already
 * over-budget; skip the sleep rather than sleeping a negative amount.
 */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * color_init() — define N_COLORS color pairs.
 *
 * Color pairs must be defined before any wattron(COLOR_PAIR(n)) call.
 * init_pair(id, fg, bg) — id 1-based (0 is reserved for default).
 *
 * 256-color path: uses xterm-256 color indices for vivid saturated colors.
 * 8-color  path: falls back to the 8 basic terminal colors.
 *
 * Pairs defined:
 *   1 → red        4 → green
 *   2 → orange     5 → cyan
 *   3 → yellow     6 → blue
 *                  7 → magenta
 */
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
        init_pair(6, 33, COLOR_BLACK);   /* blue    */
        init_pair(7, 201, COLOR_BLACK);   /* magenta */
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);   /* no orange in 8-color */
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square. A typical cell is ~2× taller than wide
 * in physical pixels (e.g. 8 px wide × 16 px tall).
 *
 * If you store a moving object's position directly in cell coordinates
 * and move it by dx=1, dy=1 per tick, it travels twice as far
 * horizontally as vertically in physical pixels. Diagonal motion looks
 * skewed. Circles become ellipses. Angles look wrong.
 *
 * THE FIX — two spaces, one conversion point:
 *
 *   PIXEL SPACE  (physics lives here)
 *     Square grid. One unit ≈ one physical pixel.
 *     Width  = cols × CELL_W   (e.g. 200 cols × 8  = 1600 px)
 *     Height = rows × CELL_H   (e.g.  50 rows × 16 =  800 px)
 *     All positions, velocities, forces in pixel units.
 *     Speed is isotropic — 1 px/s is the same distance in X and Y.
 *
 *   CELL SPACE   (drawing happens here)
 *     Terminal columns and rows.
 *     cell_x = px_to_cell_x(pixel_x)
 *     cell_y = px_to_cell_y(pixel_y)
 *     Only scene_draw() ever calls px_to_cell_x/y.
 *     Physics code never sees cell coordinates.
 *
 * WHEN §4 IS NOT NEEDED
 * ─────────────────────
 * Simulations whose "physics grid" IS the cell grid (fire, sand, this
 * demo) can work directly in cell coordinates and skip pixel↔cell
 * conversion.  §4 is retained in this template for completeness — it
 * is the first thing you add when introducing continuous motion.
 *
 * CELL_W, CELL_H
 * ──────────────
 * Logical sub-pixel steps per terminal cell.
 * CELL_H / CELL_W must match the terminal cell aspect ratio (≈ 2.0).
 * With CELL_W=8 CELL_H=16: a 200×50 terminal → pixel space 1600×800.
 */
#define CELL_W   8
#define CELL_H  16

static inline int pw(int cols) { return cols * CELL_W; }   /* pixel width  */
static inline int ph(int rows) { return rows * CELL_H; }   /* pixel height */

/*
 * px_to_cell_x/y — convert pixel coordinate to terminal cell index.
 *
 * We use  floorf(px/CELL_W + 0.5f)  — "round half up" — not roundf().
 *
 * WHY NOT roundf:
 *   C's roundf uses "round half to even" (banker's rounding).
 *   When px/CELL_W lands exactly on 0.5, it can round to 0 on one
 *   call and to 1 on the next depending on FPU state. A slow-moving
 *   object sitting on a cell boundary oscillates every frame → flicker.
 *
 * WHY NOT truncation  (int)(px/CELL_W):
 *   Always rounds down. Creates asymmetric dwell time → staircase.
 *
 * WHY floorf(px/CELL_W + 0.5f):
 *   Adds 0.5 before flooring → "round half up".
 *   Always deterministic, breaks ties in one direction, symmetric dwell.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — KeyGen                                                    */
/* ===================================================================== */

/*
 * KeyGen — state for the key-generator animation.
 *
 * Replaces Ball / Particle / etc. from other animations.
 * The "physics" here is purely timer-driven character replacement —
 * no position, no velocity, no forces.
 *
 * Fields:
 *   slots[]       current displayed character for each key position
 *   timers[]      seconds remaining until next character change per slot
 *   rates[]       base changes-per-second for each slot (randomised once)
 *   colors[]      ncurses color pair index (1–N_COLORS) per slot
 *   speed_scale   global multiplier applied to all rates (+ / - keys)
 *   paused        when true, keygen_tick() is a no-op
 */
typedef struct {
    char  slots[KEY_LEN];
    float timers[KEY_LEN];
    float rates[KEY_LEN];
    int   colors[KEY_LEN];
    float speed_scale;
    bool  paused;
} KeyGen;

/*
 * keygen_spawn() — initialise (or re-randomise) all KEY_LEN slots.
 *
 * Each slot gets:
 *   • a random printable ASCII starting character
 *   • a random cycling rate in [RATE_MIN, RATE_MAX]
 *   • a color cycling through the 7 defined pairs
 *   • a timer seeded to 1/rate so they don't all flip at frame 1
 *
 * Called on startup and when the user presses 'r'.
 */
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

/*
 * keygen_tick() — advance the animation by one fixed timestep dt (seconds).
 *
 * Equivalent to ball_tick() in bounce_ball.c.
 * Operates only on KeyGen state; has no knowledge of screen dimensions.
 *
 * For each slot:
 *   • decrement timer by (dt × speed_scale)
 *   • when timer expires: pick new random char, reset timer
 *
 * The timer reset uses the current speed_scale so that + / - key
 * changes take effect immediately on the next expiry.
 */
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

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — the collection of all entities for this animation.
 *
 * In bounce_ball.c this holds Ball balls[BALLS_MAX] + count + paused.
 * Here it holds a single KeyGen.  The struct exists so that scene_tick
 * and scene_draw have a stable signature regardless of what is inside.
 */
typedef struct {
    KeyGen kg;
} Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    keygen_spawn(&s->kg);
}

/*
 * scene_tick() — advance the simulation by one fixed-size step.
 *
 * Called from the accumulator loop in §8. dt is the fixed tick duration
 * in seconds (= 1 / sim_fps).
 *
 * cols, rows are passed here in case the entity needs pixel boundaries
 * (see bounce_ball.c scene_tick which calls pw/ph). KeyGen does not use
 * them — they are accepted but ignored to keep the signature uniform.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;   /* unused — no pixel-space physics here */
    keygen_tick(&s->kg, dt);
}

/*
 * scene_draw() — render the current scene into WINDOW *w.
 *
 * alpha ∈ [0.0, 1.0) is the render interpolation factor:
 *   alpha = sim_accum / tick_ns
 *
 * For continuous-motion entities (balls, pendulums) alpha is used to
 * extrapolate the draw position between physics ticks, eliminating
 * micro-stutter.  Example from bounce_ball.c:
 *
 *   float draw_px = b->px + b->vx * alpha * dt_sec;
 *
 * For this demo the key is stationary on screen; alpha is accepted
 * but not applied to any position. It is always part of the signature.
 *
 * Layout centered on screen:
 *
 *   row - 2  :    < GENERATING KEY >            ← label (green bold)
 *   row - 1  :    (blank)
 *   row      :    A 3 $ K 2 p ! Q 8 v ...       ← 26 cycling chars
 *   row + 1  :    - - - - - - - - - - - - -     ← dim separator
 *   row + 2  :    KEY-256  AES/RSA HYBRID        ← descriptor
 *
 * The centering arithmetic is the only place in §6 that computes
 * cell coordinates.  (No px_to_cell call needed — position is derived
 * directly from cols/rows, not from pixel physics.)
 *
 * This is the ONLY function that should call mvwaddch / mvwprintw.
 * Never draw from keygen_tick or any §5 function.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;   /* no interpolation for static entity */

    const KeyGen *k = &s->kg;

    /* Each slot is drawn with a 1-column gap between chars (2 cols/slot).
     * Total width of key row: KEY_LEN*2 - 1  (no trailing space). */
    int key_width = KEY_LEN * 2 - 1;
    int key_row   = rows / 2;
    int key_col   = (cols - key_width) / 2;
    if (key_col < 0) key_col = 0;

    /* ── label ── */
    const char *label = "< GENERATING KEY >";
    int label_len = (int)strlen(label);
    int label_col = (cols - label_len) / 2;
    if (label_col < 0) label_col = 0;

    wattron(w, COLOR_PAIR(4) | A_BOLD);
    mvwprintw(w, key_row - 2, label_col, "%s", label);
    wattroff(w, COLOR_PAIR(4) | A_BOLD);

    /* ── 26 cycling character slots ── */
    for (int i = 0; i < KEY_LEN; i++) {
        int sx = key_col + i * 2;
        if (sx < 0 || sx >= cols) continue;

        wattron(w, COLOR_PAIR(k->colors[i]) | A_BOLD);
        mvwaddch(w, key_row, sx, (chtype)(unsigned char)k->slots[i]);
        wattroff(w, COLOR_PAIR(k->colors[i]) | A_BOLD);
    }

    /* ── separator line ── */
    wattron(w, COLOR_PAIR(6) | A_DIM);
    for (int i = 0; i < key_width; i++) {
        int sx = key_col + i;
        if (sx >= 0 && sx < cols)
            mvwaddch(w, key_row + 1, sx, '-');
    }
    wattroff(w, COLOR_PAIR(6) | A_DIM);

    /* ── descriptor label ── */
    const char *desc = "KEY-256  AES/RSA HYBRID";
    int desc_len = (int)strlen(desc);
    int desc_col = (cols - desc_len) / 2;
    if (desc_col < 0) desc_col = 0;

    wattron(w, COLOR_PAIR(5) | A_DIM);
    mvwprintw(w, key_row + 2, desc_col, "%s", desc);
    wattroff(w, COLOR_PAIR(5) | A_DIM);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * ARCHITECTURE: ONE window (stdscr), ONE flush per frame (doupdate).
 *
 * ncurses maintains two virtual screens internally:
 *   curscr — what ncurses believes is currently on the physical terminal
 *   newscr — the target frame you are building right now
 *
 * Every mvwaddch / werase / wattron writes into newscr.
 * doupdate() diffs newscr vs curscr, sends only changed cells to the
 * terminal fd, then sets curscr = newscr.  THIS IS the double buffer —
 * it is always present, managed by ncurses, and is not optional.
 *
 * Common mistake — adding your own back/front WINDOW pair:
 *   Creating a second WINDOW and blitting it to stdscr introduces a
 *   third virtual screen that ncurses does not track.  The diff engine
 *   loses accuracy and you get ghost trails and torn frames.
 *
 * CORRECT FRAME SEQUENCE:
 *   erase()              — clear newscr (write spaces everywhere)
 *   scene_draw(…)        — write scene content into newscr
 *   mvprintw(…) HUD      — write status bar last so it is always on top
 *   wnoutrefresh(stdscr) — mark newscr ready; no terminal I/O yet
 *   doupdate()           — ONE write: send the diff to the terminal fd
 *
 * Never call refresh() (= wrefresh(stdscr) = mark + flush in one step).
 * If you have multiple windows to flush in one frame, call wnoutrefresh
 * on each and then ONE doupdate() at the end.  That way the terminal
 * never sees a partial frame.
 *
 * typeahead(-1): disables ncurses' habit of calling read() on stdin to
 * look for escape sequences mid-output.  Without it, output can be
 * interrupted and incomplete frames are sent.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();            /* don't echo typed characters               */
    cbreak();            /* pass keys immediately, no line buffering  */
    curs_set(0);         /* hide the hardware cursor                  */
    nodelay(stdscr, TRUE);   /* getch() returns ERR immediately if no key */
    keypad(stdscr, TRUE);    /* enable function/arrow key sequences       */
    typeahead(-1);           /* never interrupt output to peek at stdin   */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();   /* restore terminal state: show cursor, re-enable echo, etc. */
}

/*
 * screen_resize() — handle SIGWINCH (terminal resize).
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from
 * the kernel, resizing its internal virtual screens to match the new
 * terminal dimensions.  Without this, stdscr still thinks it is the
 * old size and mvwaddch at large (col, row) silently fails.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — build the complete frame in stdscr (newscr).
 *
 * Order matters:
 *   1. erase()      — blank newscr so stale content becomes spaces
 *   2. scene_draw() — write animation content
 *   3. HUD          — written last so it always renders on top
 *
 * Nothing reaches the terminal until screen_present() is called.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — top-right corner; always drawn after scene so it is on top */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  spd:%.2fx  %s ",
             fps, sim_fps, sc->kg.speed_scale,
             sc->kg.paused ? "PAUSED " : "running");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    /* key hint — bottom-left */
    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
}

/*
 * screen_present() — flush newscr to the terminal (one write).
 *
 * wnoutrefresh(stdscr) copies stdscr's content into ncurses' newscr model.
 * doupdate()           diffs newscr vs curscr, sends only changed cells.
 *
 * This is the correct two-step flush. Never just call refresh().
 */
static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state.
 *
 * g_app is a global so that signal handlers can reach it without
 * needing a pointer argument (signal handlers have a fixed signature).
 *
 * running and need_resize are volatile sig_atomic_t because they are
 * written by signal handlers and read by the main loop.  sig_atomic_t
 * is the only integer type guaranteed to be read/written atomically
 * from a signal handler on POSIX systems.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }   /* atexit safety net */

/*
 * app_do_resize() — handle a pending SIGWINCH.
 *
 * Re-reads terminal dimensions into Screen.  For animations with
 * physics in pixel space (bounce_ball.c), this also clamps entity
 * positions so they don't escape the new smaller boundary.
 * KeyGen has no positions, so only the screen dims are updated.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/*
 * app_handle_key() — process a single keypress.
 *
 * Returns false to signal "quit", true to continue.
 * All user-facing controls are handled here in one place.
 */
static bool app_handle_key(App *app, int ch)
{
    KeyGen *k = &app->scene.kg;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        k->paused = !k->paused;
        break;

    case 'r': case 'R':
        keygen_spawn(k);
        break;

    /* Cycle speed: multiply / divide by 1.5 per keypress */
    case '=': case '+':
        k->speed_scale *= 1.5f;
        if (k->speed_scale > 16.0f) k->speed_scale = 16.0f;
        break;

    case '-':
        k->speed_scale /= 1.5f;
        if (k->speed_scale < 0.1f) k->speed_scale = 0.1f;
        break;

    /* Simulation Hz — affects fixed timestep granularity */
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;

    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop
 *
 * This loop is identical in structure for every animation in the project.
 * The only things that change per-animation are scene_init/tick/draw.
 *
 * Loop body walk-through:
 *
 *   RESIZE CHECK
 *     Handle a pending SIGWINCH before touching any ncurses state.
 *     Reset frame_time and sim_accum so the accumulated dt doesn't
 *     inject a physics jump after the resize.
 *
 *   DT MEASUREMENT
 *     dt = wall-clock nanoseconds since last frame.
 *     Cap at 100 ms to prevent a physics avalanche if the process was
 *     suspended (debugger, Ctrl-Z) and then resumed.
 *
 *   SIM ACCUMULATOR (fixed timestep)
 *     sim_accum is a nanosecond "bucket".
 *     Each frame, dt is added to the bucket.
 *     While the bucket holds ≥ one tick's worth, fire one physics step
 *     and drain that tick's worth.  The remainder stays for next frame.
 *     Result: physics runs at exactly sim_fps Hz on average, regardless
 *     of render frame rate.
 *
 *   ALPHA (render interpolation)
 *     After draining, sim_accum holds the leftover time — how far we
 *     are into the NEXT tick that has not fired yet.
 *     alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to scene_draw so positions can be extrapolated to "now"
 *     rather than drawn at "last tick".  Eliminates micro-stutter.
 *
 *   FPS COUNTER
 *     Counts frames over a 500 ms window.  Divide frame count by
 *     elapsed seconds → smoothed fps estimate.  Avoids per-frame
 *     division which would oscillate wildly on fast loops.
 *
 *   FRAME CAP — SLEEP BEFORE RENDER
 *     Sleep the remaining 60fps budget BEFORE terminal I/O.
 *     If slept after, the I/O time is included in "elapsed" and
 *     the loop runs full-speed on slow terminals.
 *
 *   DRAW + PRESENT
 *     erase → scene_draw → HUD → wnoutrefresh → doupdate
 *     One atomic diff write to terminal. No partial frames.
 *
 *   INPUT
 *     Non-blocking getch() after the render.  Returns ERR immediately
 *     if no key is pending (nodelay is TRUE from screen_init).
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed the RNG from the monotonic clock so each run looks different */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Register atexit handler as a safety net in case endwin() is missed */
    atexit(cleanup);

    /* SIGINT / SIGTERM — set running=0 to exit the loop gracefully      */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — set need_resize=1; handled at top of next loop iter    */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();   /* timestamp of last frame start */
    int64_t sim_accum   = 0;            /* nanoseconds in the bucket     */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window */
    int     frame_count = 0;            /* frames in current fps window  */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD     */

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* pause guard */

        /* ── sim accumulator (fixed timestep) ────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── alpha — render interpolation factor ─────────────────── */
        /*
         * sim_accum is now the leftover ns after all full ticks.
         * alpha ∈ [0, 1) indicates how far we are into the next tick.
         * Pass to scene_draw so entities can be drawn at "now" not
         * "last tick" — eliminates visible stutter between ticks.
         *
         * For a paused scene, zero alpha for pixel-perfect freeze:
         *   float alpha = app->scene.kg.paused ? 0.0f :
         *                 (float)sim_accum / (float)tick_ns;
         */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter (500 ms sliding window) ─────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap — sleep BEFORE render ─────────────────────── */
        /*
         * elapsed = time spent on physics since frame_time was updated.
         * Budget  = NS_PER_SEC / 60  (one 60fps frame in ns).
         * Sleep   = budget − elapsed.
         *
         * Sleeping BEFORE render means only physics time is charged
         * against the budget.  Terminal I/O (doupdate, getch) happens
         * after the sleep and does not affect the next frame's timing.
         */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
