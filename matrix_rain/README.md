# matrix_rain — falling glyphs that reroll every frame

A reference for the **Matrix-style rain** variants in this project. This
folder contains **5 self-contained C programs**, each taking the same
core trick — a head glyph followed by a fading trail of random ASCII
characters that **reroll every tick** — and applying it to a different
trajectory: vertical fall, parabolic arc, rotating beam, radial corona,
or rain accumulating into snow.

Every file in this folder is built around the **same primitive**:
a fixed-length **shimmer cache** of glyphs per stream, slid forward one
slot per tick, with most slots rerolled to a new random character. The
trajectory changes; the cache discipline is identical.

If you read **only one file**, read
[`matrix_rain.c`](matrix_rain.c) — it is the canonical exemplar (pure
vertical fall) and every other file in the folder declares itself as a
variation of it in its CONCEPTS block.

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
3. [Per-file table](#per-file-table)
4. [Building and running](#building-and-running)

---

## How to read this folder

The recommended path goes from **simplest trajectory** (straight down)
to **most complex** (rain that freezes into snow). Each file changes
exactly one thing about the previous.

```
   1.  matrix_rain.c                  vertical fall, fixed columns
        the base shimmer cache
                │
                ▼
   2.  fireworks_rain.c               parabolic arc + per-spark cache
        trajectory becomes ballistic
                │
                ▼
   3.  pulsar_rain.c                  rotating beam + per-beam wake
        trajectory becomes angular sweep
                │
                ▼
   4.  sun_rain.c                     radial outward + per-ray cache
        trajectory becomes 360° spokes
                │
                ▼
   5.  matrix_snowflake.c             vertical fall + accumulation
        adds a second simulation (pile growth) on top of base rain
```

**Prerequisites graph.** Every file's header has a *Study alongside:*
block pointing at siblings:

* `matrix_rain.c` is the root. Every other file describes itself as
  "the shimmer-cache trick applied to <new trajectory>".
* `fireworks_rain.c` is the most physically interesting variant — the
  cache rides a ballistic arc with gravity, so the trail traces the
  exact path of a projectile.
* `pulsar_rain.c` and `sun_rain.c` are the **rotating** and **radial**
  cousins respectively. The pulsar's beam sweeps; the sun's rays slide
  along fixed angles. If you set the pulsar's `omega = 0` you get
  sun_rain.
* `matrix_snowflake.c` is the only file with **two simulations** —
  falling rain on top, frozen snow pile underneath, with a per-column
  boundary that grows upward as the rain hits it.

---

## The unifying primitive

Every file in this folder maintains, per stream, a **fixed-length
circular cache of glyphs** that slides forward one slot per tick. The
front of the cache is the bright HEAD, the back is the FADE tail.

```c
/* Sketch — every file's §4 entity has something like: */
typedef struct {
    /* trajectory state — changes per file */
    float px, py;                /* head position */
    float vx, vy;                /* head velocity (where relevant) */

    /* shimmer cache — IDENTICAL discipline across files */
    char  cache[TRAIL_LEN];      /* random glyphs, [0] = head */
    int   shade[TRAIL_LEN];      /* HOT / WARM / COOL / FADE band index */
} Stream;
```

The tick is **three lines**:

```c
static void stream_tick(Stream *s, float dt) {
    advance_trajectory(s, dt);   /* per-file: gravity, rotation, ... */
    slide_cache_back(s);         /* cache[k] = cache[k-1] for k=N..1 */
    s->cache[0] = random_glyph();/* fresh glyph at the head */
    reroll_some_tail_slots(s);   /* not all — preserves visual coherence */
}
```

The visual effect — **the same falling trajectory looks like
continuous shimmer instead of a static curve** — comes from the cache
reroll. Without it, every column would draw the same eight glyphs
every frame and look like a fixed barber's pole. With it, the glyphs
keep changing but the *envelope* (the head's path) is steady, so your
eye sees the path while your retina sees flickering letters.

Three things follow from the cache representation:

1. **Trajectory and rendering decouple.** Change the trajectory (rain
   → arc → beam → ray → accumulating pile) without touching the cache
   logic. The five files are literally a 1-axis substitution.
2. **Fade is a band index, not a colour.** Each cache slot stores a
   band index `0..4` (HOT, WARM, MID, COOL, FADE) and looks up the
   actual ncurses pair via the active theme. Switching themes (`t`)
   changes nothing about the simulation — only the colour mapping.
3. **The reroll rate is a knob.** Reroll every slot every tick → pure
   noise, no readable letters. Reroll only the head → static text on
   a moving curve. Most files reroll 25–50% of slots per tick — enough
   for shimmer, low enough to keep letterforms visible.

Layout, drawn flat (one column, time-frozen, head at bottom):

```
            col_x
              │
         ┌── row 0
         │     b   ← cache[7]   FADE   dim
         │     k   ← cache[6]   FADE   dim
         │     7   ← cache[5]   COOL
         │     2   ← cache[4]   MID
         │     Q   ← cache[3]   MID
         │     m   ← cache[2]   WARM   bold
         │     g   ← cache[1]   HOT    bold
         ▼     A   ← cache[0]   HEAD   white bold
            row N
```

The HEAD is **always pure white** across all five files — it's the
single visual constant that lets your eye track the leading edge of
each stream even as the colour band underneath changes by theme.

---

## Per-file table

| File                  | Trajectory                         | Stream count               | What's new vs. base                                 |
|-----------------------|------------------------------------|----------------------------|-----------------------------------------------------|
| `matrix_rain.c`       | vertical fall, `vy` only           | one per terminal column    | the base shimmer cache, four themes (green/amber/blue/white) |
| `fireworks_rain.c`    | parabolic arc, `vy += g·dt`        | up to ~12 rockets × 72 sparks | gravity integration; rocket state machine IDLE→RISING→EXPLODED |
| `pulsar_rain.c`       | angular sweep, `θ += ω·dt`         | 1..16 beams (default 2)    | rotation; angular wake spans `WAKE_LEN · WAKE_STEP` degrees |
| `sun_rain.c`          | radial outward, fixed angle per ray| 180 rays from one centre   | 360° spokes; no rotation — each ray slides along its own angle |
| `matrix_snowflake.c`  | vertical fall + freeze-on-pile     | one per column + pile      | second simulation underneath: pile grows when stream head reaches its top; world resets when full |

**Visual signatures** (what you see distinguishes which file is running):

* `matrix_rain.c` — vertical curtains, columns independent.
* `fireworks_rain.c` — bursts of arcs rising and falling under gravity.
* `pulsar_rain.c` — a single central `@` with rotating lighthouse
  beams; 2 beams = classic pulsar pair, 4 = cross, etc.
* `sun_rain.c` — a single central `@` with **non-rotating** rays
  shooting outward in all directions.
* `matrix_snowflake.c` — rain on top, growing snow pile beneath, with
  a ragged boundary that climbs as the simulation runs.

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra <file>.c -o <name> -lncurses -lm
```

All five files need both `-lncurses` and `-lm` (trigonometry in the
rotating / radial variants, `expf` for fade ramps everywhere).

**Universal keys** (present in every file):

| Key             | Action                                       |
|-----------------|----------------------------------------------|
| `q` / `Q` / `ESC` | quit                                       |
| `space` / `p`   | pause / resume                               |
| `r`             | reset all streams                            |
| `t` / `T`       | cycle theme forward / back                   |
| `+` / `-`       | speed up / slow down                         |
| `]` / `[`       | sim Hz up / down                             |

**Per-file specials**:

| File                  | Specific keys                                       |
|-----------------------|-----------------------------------------------------|
| `pulsar_rain.c`       | `]` / `[` add / remove beam (1..16 evenly spaced)   |
| `fireworks_rain.c`    | space pause; rockets auto-spawn                     |
| `matrix_snowflake.c`  | the full-pile flash auto-resets every ~1 second     |

See [`particle_systems/`](../particle_systems/) for the underlying
rocket-and-burst skeleton without trails (`fireworks.c`) and for
non-shimmer particle systems generally. See
[`grids/`](../grids/README.md) if you want to understand the cell-space
vs pixel-space discipline that the matrix-rain files inherit from.
