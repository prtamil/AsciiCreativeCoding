# Color Techniques

A catalogue of the colour tricks used across this project — how to coax rich,
smooth colour out of a terminal that only knows letters and a fixed palette.
Each entry is *what the trick is*, *why you'd reach for it*, and the *core
formula or code*, with a few files that use it. For a demo's full context, read
its own header; this is the grab-bag of reusable ideas.

The tricks split into five questions:
1. **Palette plumbing** — getting colours onto the screen at all.
2. **Value → shade** — turning one number into a glyph or brightness.
3. **Dithering** — hiding the steps between a handful of levels.
4. **Making palettes & gradients** — generating the colours themselves.
5. **Colouring *by* something** — what quantity drives the colour.

---

## 1 · Palette plumbing

### 256-colour cube, with an 8-colour fallback
**What / why.** Rich terminals expose 256 colours; old/SSH ones expose 8. Check
once at startup and pick the right indices, so the *same binary* looks good on
both.
The 256 palette is: 0–15 the basic ANSI colours, **16–231 a 6×6×6 RGB cube**,
**232–255 a 24-step grey ramp**. A cube colour from 0–5 components is:
```c
int cube = 16 + 36*r5 + 6*g5 + b5;          /* r5,g5,b5 ∈ 0..5 */
if (COLORS >= 256) init_pair(1, cube, -1);
else               init_pair(1, COLOR_RED, -1);   /* graceful fallback */
```
*Used in:* fire.c, fireworks.c, flocking.c, bounce_ball.c.

### A_BOLD / A_DIM as free brightness tiers
**What / why.** One colour pair can read as *three* brightnesses — most
terminals render `A_DIM`/`A_BOLD` as darker/brighter rather than font weight. A
cheap way to add depth without spending pairs (vital on 8-colour terminals).
```c
attr_t a = COLOR_PAIR(p);
if      (life > 0.6f) a |= A_BOLD;   /* bright */
else if (life < 0.2f) a |= A_DIM;    /* faint  */
```
*Used in:* fire.c, fireworks.c, donut.c, flocking.c.

### Transparent background — `use_default_colors()`
**What / why.** Call it right after `start_color()` and `-1` becomes a legal
colour meaning "the terminal's own background." Glyphs then float over whatever
wallpaper/colour the terminal has, instead of sitting in a black box.
```c
start_color(); use_default_colors();
init_pair(1, 130, -1);               /* fg=brown, bg=transparent */
```
*Used in:* bonsai.c, matrix_rain/*.

### Live palette switching — the theme struct
**What / why.** Keep each theme as a small array of colour indices; switching
themes just re-runs `init_pair()` with the new indices. The drawing code never
changes — only the palette — so a `t` keypress recolours the whole scene.
```c
void theme_apply(int t){ for (int i=0;i<RAMP_N;i++) init_pair(CP_BASE+i, themes[t].fg[i], -1); }
```
*Used in:* fire.c, smoke.c, and every artistic demo with `t`-cycling themes.

### The bright-half-of-256 legibility rule
**What / why.** The bottom of the cube (16–23) and the dark greys (232–239) are
nearly invisible on a black background — lethal with `A_DIM`. Keep *every*
palette entry, even the "darkest" tier, in the bright half (30+/244+). Character
comes from the *relative* gradient, not absolute darkness, so nudging `ramp[0]`
from 17→24 keeps the look and gains visibility.

### Colour-pair budget
**What / why.** ncurses pairs are a small numbered resource. Reserve fixed ranges
up front — e.g. HUD pairs, then a contiguous block per gradient (`PAIR_LAVA_BASE
+ 0..5`) — and index into them by slot. One allocation scheme per program keeps
the renderer from ever guessing a pair number.

---

## 2 · Value → shade (one number becomes a glyph)

### Paul Bourke density ramp
**What / why.** The workhorse: turn a brightness in `[0,1]` into a character by
ink coverage. Bourke's 94-char string is ordered light→heavy, giving 94 shades
from glyphs alone.
```c
static const char k_bourke[] =
 " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
char ch = k_bourke[(int)floorf(luma * (sizeof k_bourke - 2))];
```
*Used in:* donut.c, sphere_raster.c, cube_raster.c, torus_raster.c.

### LUT quantization with perceptual break spacing
**What / why.** Map a continuous value to a short ramp through a *lookup table of
break points* — and make the breaks **uneven**, clustered where the eye cares
most (the mid-tones of a flame), coarse in near-black/near-white. O(1) per cell,
and the gradient reads smooth where it matters.
```c
static const float breaks[] = { 0,.08f,.18f,.29f,.39f,.50f,.62f,.75f,.90f };
static const char  ramp[]   = " .:+x*X#@";
/* find the slot whose break the value just passed */
```
*Used in:* fire.c, aafire_port.c.

### Gamma correction
**What / why.** Physical brightness is linear; human vision isn't. Raise to
`1/2.2` (sRGB) before mapping to a glyph/colour, or darks get crushed and brights
wash out.
```c
float shown = powf(linear, 1.0f/2.2f);
```
*Used in:* fire.c, the `raster/` shaders.

### Grey luminance ramp
**What / why.** Pure brightness shading with no hue: pick ~8 indices spread
across the 232–255 grey ramp and draw the *same* glyph, changing only the pair.
```c
int greys[8] = {235,238,241,244,247,250,253,255};
```
*Used in:* donut.c, sphere_raster.c, cube_raster.c.

---

## 3 · Dithering (smoothing the steps)

### Bayer 4×4 ordered dithering
**What / why.** With only a few brightness levels, flat regions band. Add a fixed
4×4 threshold (keyed off pixel position) before quantizing, and the levels
interleave into a stable crosshatch that *reads* as an in-between shade. Cheap,
no neighbour bookkeeping — ideal for spinning 3-D surfaces.
```c
static const float bayer[4][4] = {
  { 0/16.f, 8/16.f, 2/16.f,10/16.f},{12/16.f, 4/16.f,14/16.f, 6/16.f},
  { 3/16.f,11/16.f, 1/16.f, 9/16.f},{15/16.f, 7/16.f,13/16.f, 5/16.f}};
float d = luma + (bayer[py&3][px&3] - 0.5f)*strength;
```
*Used in:* sphere_raster.c, cube_raster.c, torus_raster.c, displace_raster.c.

### Floyd-Steinberg error diffusion
**What / why.** The other dithering route: after picking the nearest level, push
the leftover error onto not-yet-drawn neighbours (7/16 right, 3/16/5/16/1/16
below). Smoother than Bayer for soft gradients (fire), at the cost of a scratch
buffer and order-dependence.
```c
float err = v - quantized;
right += err*7/16.f;  below_left += err*3/16.f;  below += err*5/16.f;  below_right += err*1/16.f;
```
*Used in:* fire.c, aafire_port.c.

---

## 4 · Making palettes & gradients

### Cosine gradient palettes (Iñigo Quílez)
**What / why.** A whole smooth palette from one cheap formula — no gradient
texture, no table. Each channel is a cosine; shifting the phase animates the hue,
and offsetting phases per object keeps them distinct.
```
channel(t) = 0.5 + 0.5 * cos( 2π * (t + phase) )      /* per R,G,B */
```
Three flocks at phases 0, ⅓, ⅔ stay 120° apart forever. Quantize the float RGB
back to the cube (`16 + 36·r5 + 6·g5 + b5`). For a still gradient, **pre-bake**
the formula into a small LUT once and just index it each frame.
*Used in:* flocking.c, complex_flowfield.c.

### HSV → RGB
**What / why.** When the thing you want to vary is *hue* (iteration depth, an
angle, a category), build the colour in HSV and convert. One scalar sweeps the
rainbow without hand-listing colours.
```c
/* h∈[0,360), s,v∈[0,1] → r,g,b; standard sextant conversion */
```
*Used in:* mandelbulb_raster.c, the escape-time fractals.

### Linear gradient interpolation (lerp)
**What / why.** Blend between a few hand-picked key colours: pick the two
surrounding stops for `t` and `lerp` each channel. The simplest way to author a
controlled ramp (sky horizons, heat ramps).
```c
out = a + (b - a) * frac;     /* per channel, between key colours a,b */
```
*Used in:* theme palettes throughout (sky/lava/plume ramps).

### Diverging / signed colormap
**What / why.** For quantities that go negative *and* positive (pressure,
amplitude, a Lyapunov exponent): map negative to one hue, positive to another,
and zero to a neutral midpoint — so sign and magnitude are both readable at a
glance.
```
v < 0 → cool ramp by |v|      v > 0 → warm ramp by v      v ≈ 0 → neutral
```
*Used in:* acoustic_wavesolver.c, vorticity_streamfunction_solver.c, lyapunov.c.

### Blackbody / temperature (Kelvin) ramp
**What / why.** Physically-motivated warmth: drive the whole scene's colour from
a single temperature, walking the Planckian locus from deep red (cool) through
orange/white to blue (hot). One Kelvin value sets shaft, glow, and sky together.
*Used in:* forest_god_rays.c / god_rays_window.c, sun_solar.c, blackhole.c.

---

## 5 · Colouring *by* something

### Escape-time / smooth-iteration (fractals)
**What / why.** Colour each point by *how long* its iteration took to escape.
The raw integer count bands badly; the smooth (fractional) count removes the
rings:
```
μ = n + 1 - log2( log|z| )        /* continuous escape value → palette */
```
*Used in:* mandelbrot, julia, burning_ship, newton_fractal (basin + speed).

### Density / log-density accumulator
**What / why.** When many samples pile into the same cell (Buddhabrot, DLA,
N-body glow), colour by *how many* landed there. Counts are wildly skewed, so map
`log(1 + count)`, normalised, into the palette — otherwise a few hot cells crush
everything else to black.
*Used in:* buddhabrot.c, barnes_hut.c, snowflake.c / coral.c (DLA).

### Distance / proximity brightness
**What / why.** Brighten things near a focus: compute a distance, and inside a
radius bump to `A_BOLD`. A density/depth cue using a single attribute bit, no
extra pairs.
*Used in:* flocking.c (boids near their leader), constellation.c.

### Velocity / angle → hue
**What / why.** Encode *direction* as colour: turn a velocity angle into an
octant (or a continuous hue) so the picture shows which way the flow is going.
```c
int octant = (int)((atan2f(-vy,vx) + M_PI)/(M_PI/4)) % 8;   /* → 1 of 8 hue pairs */
```
*Used in:* flowfield.c, complex_flowfield.c.

### Surface normal → luminance (3-D shading)
**What / why.** The classic lighting step: dot the surface normal with a light
direction to get brightness, then run that through the Bourke ramp. This is what
makes a sphere look round.
```c
float lum = fmaxf(0.f, dot(normal, light));   /* → glyph via density ramp */
```
*Used in:* the `raster/` shaders, the raymarchers (`raymarcher/*`).

### Depth / recursion level → hue
**What / why.** In recursive/branching art, let nesting depth pick the colour, so
the structure's *generation* is visible — each Koch level, each Apollonian
circle, each lightning fork in its own band of the spectrum.
*Used in:* koch.c, apollonian.c, lightning.c.

---

*All of these sit on top of the same two primitives: an ncurses colour pair
(`init_pair`) and a glyph chosen from a density ramp. Everything above is just a
different answer to "what colour, and how bright, should this cell be?"*
