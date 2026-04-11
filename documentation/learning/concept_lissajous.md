# Pass 1 — artistic/lissajous.c: Harmonograph / Lissajous Figures

## Core Idea

Two perpendicular damped sinusoidal oscillators trace a parametric curve on screen. Their frequency ratio determines the figure's topology (figure-8, trefoil, star…) and a slowly drifting phase offset morphs the shape continuously. After a full 2π phase sweep the program advances to the next rational frequency ratio and the process repeats.

## The Mental Model

Imagine two pendulums: one swinging left-right, one swinging toward you. Attach a pen to their crossing point and it traces a Lissajous figure. Add spring damping — the oscillations decay — so the pen spirals inward, tracing a layered shell of overlapping curves. A harmonograph is the physical instrument; `lissajous.c` is its mathematical recreation.

## Parametric Equations

```
x(t) = sin(fx·t + phase) · exp(−decay·t)
y(t) = sin(fy·t)         · exp(−decay·t)
```

- `fx`, `fy` — frequency ratio (e.g. fx=2, fy=3 for Trefoil)
- `phase` — phase offset of the x oscillator; drifts slowly each frame
- `decay` — exponential attenuation; controls how fast the spiral winds inward

At `t=0`: full amplitude outer ring. As `t → T_MAX`: amplitude → ~1%, innermost point.

## T_MAX and DECAY Normalization

```c
float t_max = N_LOOPS * 2π / min(fx, fy);
float decay = DECAY_TOTAL / t_max;
```

`T_MAX` is chosen so the program always draws exactly `N_LOOPS=4` complete cycles of the *slower* oscillator — regardless of the frequency ratio. `DECAY` is then derived so amplitude reaches `exp(−DECAY_TOTAL) ≈ 1%` at that `T_MAX`.

**Why this matters:** Without normalization, a 1:2 ratio would draw a shallow spiral and a 3:5 ratio would draw a deep one. With normalization every ratio produces the same visual spiral depth.

## Age-Based Rendering

The curve is sampled at `N_CURVE_PTS=2500` parametric points. Drawn oldest-first (innermost) so newest (outermost) overwrites shared cells:

```c
for (int i = N_CURVE_PTS - 1; i >= 0; i--) {
    float t   = (float)i / (float)(N_CURVE_PTS - 1) * t_max;
    float amp = expf(-decay * t);
    float age = (float)i / (float)(N_CURVE_PTS - 1);
    int   lev = (int)(age * N_LEVELS);   /* 0 = newest, 3 = oldest */
    ...
}
```

- `i = 0` → `t = 0` → `age = 0` → level 0 (brightest `#`, `A_BOLD`). This is the outer ring at full amplitude.
- `i = N-1` → `t = T_MAX` → `age = 1` → level 3 (dimmest `.`). This is the innermost point near zero amplitude.

The outermost bright ring always overwrites dimmer inner cells on overlap — the figure looks spatially layered without any explicit z-buffer.

## Phase Drift and Dwell Mechanism

Phase advances by `eff_drift()` each tick, not a fixed constant:

```c
float kp   = M_PI / fmaxf(fx, fy);      /* key phase period */
float pm   = fmodf(phase_x, kp);
float d    = fminf(pm, kp - pm);        /* dist to nearest symmetric phase */
float frac = d / (kp * DWELL_WIDTH);
float mul  = DWELL_SPEED + (1.0f - DWELL_SPEED) * fminf(frac, 1.0f);
```

Symmetric Lissajous figures (the closed, named shapes) occur at phase multiples of `π/max(fx,fy)`. Near these angles drift is multiplied to `DWELL_SPEED=0.25`, giving each named shape time to dwell on screen before transitioning. The linear ramp over `DWELL_WIDTH=0.25` of the key period prevents a jarring speed change.

## Frequency Ratios and Figure Topology

| Ratio | Shape | Key angle period |
|---|---|---|
| 1:2 | Figure-8 | π/2 |
| 2:3 | Trefoil | π/3 |
| 3:4 | Star | π/4 |
| 1:3 | Clover | π/3 |
| 3:5 | Pentagram | π/5 |
| 2:5 | Five-petal | π/5 |
| 4:5 | Crown | π/5 |
| 1:4 | Eye | π/4 |

After phase drifts 2π (a complete phase revolution), the ratio auto-advances to the next entry.

## Four Themes × Four Brightness Levels

```c
static const int k_theme[N_THEMES][N_LEVELS] = {
    { 226, 220, 136,  94 },   /* Golden: bright-yellow → brown      */
    {  51,  38,  23,  17 },   /* Ice:    bright-cyan  → navy        */
    { 231, 208, 196,  88 },   /* Ember:  white → red → dark-red     */
    { 118,  82,  28,  22 },   /* Neon:   bright-green → very-dark   */
};
```

16 `init_pair` calls at startup + 1 HUD pair. Theme switching is `s.theme = (s.theme+1) % N_THEMES` — no re-registration.

## Non-Obvious Decisions

### Why draw oldest first?
The outermost loop (newest, `i=0`) is at full amplitude and thus maps to the widest positions. Drawing it last means it overwrites the inner dim dots at any shared cell. This creates the impression of a lit outer ring surrounding darker inner coils — depth without a depth buffer.

### Why `phase` only on x(t)?
Applying the same phase shift to both oscillators is equivalent to a time shift — the figure just translates along the parametric curve. Applying it only to `x` genuinely deforms the shape, creating the morphing behavior.

### Why `DWELL_WIDTH = 0.25`?
The ramp occupies ±25% of the key period on either side of a symmetric phase. Too wide and the drift never reaches full speed. Too narrow and the transition is abrupt. 0.25 gives roughly half the inter-key interval at half speed, which feels natural at 30 fps.

## Key Constants

| Constant | Effect |
|---|---|
| N_CURVE_PTS | Parametric samples per frame (2500) |
| N_LOOPS | Complete cycles of slower oscillator (4) |
| DECAY_TOTAL | Total exponent → amplitude ~1% at T_MAX (4.5) |
| DRIFT_DEFAULT | Base phase advance per tick (0.004 rad ≈ 35s per ratio) |
| DWELL_SPEED | Drift multiplier at symmetric phase (0.25) |
| DWELL_WIDTH | Fraction of key period to ramp drift (0.25) |
| CELL_ASPECT | Physical char height/width ratio (2.0) |
| AMP_FRAC | Fraction of screen half-size used (0.92) |
