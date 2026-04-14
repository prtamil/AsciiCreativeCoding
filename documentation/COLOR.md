# Color Techniques

A reference for every color trick used across the C source files in this project. Each entry names the technique, explains the mechanism, and shows the code pattern.

---

## 1. 256-Color with 8-Color Fallback

**Where:** fire.c, aafire_port.c, brust.c, fireworks.c, constellation.c, bounce_ball.c, flocking.c

**How it works:**
At startup, check `COLORS >= 256`. On capable terminals, use specific xterm-256 indices for rich color. On limited terminals, fall back to standard ncurses `COLOR_*` constants. Same binary runs correctly on both xterm-256color and basic 8-color SSH sessions.

```c
if (COLORS >= 256) {
    init_pair(1, 196, COLOR_BLACK);  /* bright red */
} else {
    init_pair(1, COLOR_RED, COLOR_BLACK);
}
```

---

## 2. A_BOLD / A_DIM as Brightness Modifiers

**Where:** fire.c, aafire_port.c, fireworks.c, constellation.c, flocking.c, donut.c

**How it works:**
Bitwise OR `A_BOLD` or `A_DIM` with `COLOR_PAIR()` to shift the perceived brightness of a color without allocating extra pairs. Most terminals interpret these attributes as brightness shifts rather than font weight. A single color pair becomes three apparent levels: dim, normal, and bold.

```c
attr_t attr = COLOR_PAIR(pair_num);
if (life > 0.6f) attr |= A_BOLD;      /* bright */
else if (life < 0.2f) attr |= A_DIM;  /* dim */
attron(attr);
mvaddch(y, x, ch);
attroff(attr);
```

**Effect:** Triples apparent brightness resolution from one color pair — essential on 8-color terminals.

---

## 3. Theme Switching / Dynamic Palette Cycling

**Where:** fire.c, aafire_port.c

**How it works:**
Six predefined `FireTheme` structs each contain 9 xterm-256 color indices covering a heat gradient. Themes cycle automatically every `CYCLE_TICKS` frames or on manual `t` keypress. `theme_apply()` re-registers all color pairs with the new theme's indices — no visual structure changes, only the palette.

**Themes:**
| Name   | Colors                                |
|--------|---------------------------------------|
| fire   | Red → orange → yellow                 |
| ice    | Dark blue → cyan → white              |
| plasma | Dark purple → magenta → white         |
| nova   | Dark green → lime → white             |
| poison | Green → yellow-green → white          |
| gold   | Amber → bright yellow → white         |

```c
static void theme_apply(int t) {
    const FireTheme *th = &k_themes[t];
    for (int i = 0; i < RAMP_N; i++) {
        init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
    }
}
```

---

## 4. Floyd-Steinberg Error Diffusion Dithering

**Where:** fire.c, aafire_port.c

**How it works:**
Converts continuous float heat values [0,1] to 9-level ASCII characters by propagating quantization error to neighboring cells. After choosing the nearest ramp level for a cell, the difference between the actual and chosen value is distributed to four neighbors with fixed weights.

**Error distribution weights:**
- Right: 7/16
- Below-left: 3/16
- Below: 5/16
- Below-right: 1/16

```c
int idx = lut_index(v);
float qv = lut_midpoint(idx);
float err = v - qv;

if (x+1 < cols)
    dither[i+1]      += err * (7.f/16.f);
if (y+1 < rows) {
    dither[i+cols-1] += err * (3.f/16.f);
    dither[i+cols]   += err * (5.f/16.f);
    dither[i+cols+1] += err * (1.f/16.f);
}
```

**Effect:** Smooth gradients from continuous physics to discrete ASCII — eliminates hard banding between heat levels.

---

## 5. Gamma Correction

**Where:** fire.c, aafire_port.c, sphere_raster.c, cube_raster.c, torus_raster.c, displace_raster.c

**How it works:**
Physical brightness values are linear, but human vision is not. Applying exponent `1/2.2` (the sRGB standard) before mapping to character or color index ensures that the rendered brightness matches perceived brightness. Without correction, dark tones are crushed and bright tones wash out.

```c
float display_value = powf(linear_value, 1.f / 2.2f);
```

**Effect:** Gradients look perceptually smooth instead of skewed toward dark or light.

---

## 6. Grey Ramp for Luminance Gradients

**Where:** donut.c, sphere_raster.c, cube_raster.c, torus_raster.c

**How it works:**
xterm-256 indices 232–255 are a 24-step grey ramp from nearly black to white. Eight evenly-spaced indices are selected to create a luminance gradient. The same character is drawn at each brightness level; only the color changes.

```c
if (COLORS >= 256) {
    int greys[8] = { 235, 238, 241, 244, 247, 250, 253, 255 };
    for (int i = 0; i < 8; i++)
        init_pair(i + 1, greys[i], COLOR_BLACK);
}
```

**Effect:** 8-level brightness gradient without changing the drawn character — pure luminance shading.

---

## 7. Bayer 4×4 Ordered Dithering

**Where:** sphere_raster.c, cube_raster.c, torus_raster.c, displace_raster.c

**How it works:**
A 4×4 threshold matrix is used instead of error diffusion. For each pixel, the Bayer threshold at its position is added to the luminance value before quantizing. This creates a stable, repeating geometric pattern that approximates intermediate brightness levels.

```c
static const float k_bayer[4][4] = {
    {  0/16.f,  8/16.f,  2/16.f, 10/16.f },
    { 12/16.f,  4/16.f, 14/16.f,  6/16.f },
    {  3/16.f, 11/16.f,  1/16.f,  9/16.f },
    { 15/16.f,  7/16.f, 13/16.f,  5/16.f },
};

float thr = k_bayer[py & 3][px & 3];
float d   = luma + (thr - 0.5f) * strength;
char  ch  = k_bourke[(int)floorf(d * BOURKE_LEN)];
```

**Effect:** Cheaper than Floyd-Steinberg; produces regular crosshatch-like patterns that suit 3D surface shading.

---

## 8. Paul Bourke ASCII Density Ramp

**Where:** sphere_raster.c, cube_raster.c, torus_raster.c, displace_raster.c

**How it works:**
A hand-curated 94-character string ordered from lowest to highest visual ink density. Mapping luminance [0,1] to an index in this string gives 94 distinct brightness levels through character selection alone, without any color change.

```c
static const char k_bourke[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";

int  idx = (int)floorf(luminance * BOURKE_LEN);
char ch  = k_bourke[idx];
```

**Effect:** 94-level brightness resolution from character density — far more nuanced than short ramps like `" .:-=+*#@"`.

---

## 9. `use_default_colors()` for Transparent Background

**Where:** bonsai.c, matrix_rain.c

**How it works:**
Calling `use_default_colors()` immediately after `start_color()` enables `-1` as a valid color index in `init_pair()`. This maps to the terminal's own background color (ISO 6429 SGR 49 — background reset). Characters are then drawn floating over whatever the terminal background is, including wallpaper or images in terminals that support it.

```c
static void color_init(void) {
    start_color();
    use_default_colors();
    init_pair(1, 130, -1);  /* -1 = terminal's own background */
}
```

**Effect:** Drawn characters appear without a forced solid background — essential for decorative effects like bonsai or matrix rain where a black box would look wrong.

---

## 10. Cosine Palette Cycling (Continuous Hue Animation)

**Where:** flocking.c

**How it works:**
RGB values for each flock's color pair are recomputed every frame using the cosine formula. The frame counter `t` advances the phase, causing the color to smoothly cycle through the spectrum. Three flocks use phase offsets 0, 1/3, and 2/3 so they are always 120° apart in hue and remain visually distinct throughout the cycle.

**Formula:**
```
r = 0.5 + 0.5 * cos(2π * (t/period + phase_r))
g = 0.5 + 0.5 * cos(2π * (t/period + phase_g))
b = 0.5 + 0.5 * cos(2π * (t/period + phase_b))
```

**Mapping float RGB to xterm-256 cube index (indices 16–231):**
```c
int ri       = (int)(r * 5.0f + 0.5f);
int gi       = (int)(g * 5.0f + 0.5f);
int bi       = (int)(b * 5.0f + 0.5f);
int cube_idx = 16 + 36 * ri + 6 * gi + bi;
init_pair(pair_num, cube_idx, COLOR_BLACK);
```

**Effect:** Colors animate continuously and independently per flock without touching any boid rendering logic — the simulation is color-agnostic.

---

## 11. Proximity Brightness (Distance-Based `A_BOLD`)

**Where:** flocking.c, constellation.c

**How it works:**
Each frame, compute the toroidal distance between a follower boid and its flock leader. If the distance is within 35% of the perception radius, draw the boid with `A_BOLD`; otherwise draw it normally. This creates a brightness halo around dense clusters.

```c
static int follower_brightness(const Boid *follower, const Boid *leader,
                                float max_px, float max_py) {
    float dx    = toroidal_delta(follower->px, leader->px, max_px);
    float dy    = toroidal_delta(follower->py, leader->py, max_py);
    float ratio = hypotf(dx, dy) / PERCEPTION_RADIUS;
    return (ratio < 0.35f) ? A_BOLD : A_NORMAL;
}
```

**Effect:** Dense clusters appear brighter — a depth/density cue using only one attribute bit, no extra pairs.

---

## 12. LUT-Based Quantization with Perceptual Threshold Spacing

**Where:** fire.c, aafire_port.c

**How it works:**
A look-up table of break points maps continuous heat values to a 9-level ASCII ramp. The breaks are not evenly spaced; they are clustered in the 0.3–0.75 range where flame gradient curvature is most visible to human eyes. This gives fine resolution where it matters and coarser steps in near-black and near-white regions.

```c
static const float k_lut_breaks[] =
    { 0.000f, 0.080f, 0.180f, 0.290f, 0.390f,
      0.500f, 0.620f, 0.750f, 0.900f };

static const char k_ramp[] = " .:+x*X#@";
```

**Effect:** Smooth gradient appearance in the perceptually important mid-range without wasting levels on extremes. Fast O(1) lookup per cell.

---

## 13. Velocity/Angle-to-Color Mapping

**Where:** flowfield.c

**How it works:**
In RAINBOW theme, the particle's velocity direction is mapped to a hue pair index. `atan2(-vy, vx)` returns the angle in `[-π, π]`; adding `π` shifts to `[0, 2π]`; dividing by `π/4` gives an octant index 0–7. Each octant maps to one of 8 color pairs registered to hues spanning the spectrum. In mono themes (CYAN, GREEN, WHITE), the same 8 pairs are instead used for trail age: index 1 = newest/brightest, index 8 = oldest/dimmest. The two uses are mutually exclusive and switched per theme.

```c
/* RAINBOW theme: angle → octant → hue pair */
int octant = (int)((atan2f(-vy, vx) + M_PI) / (M_PI / 4.0f)) % 8;
int pair   = 1 + octant;          /* pairs 1..8 = 8 hue sectors */

/* Mono theme: trail position index → brightness pair */
int pair = 1 + trail_age_index;   /* 1=newest/bright, 8=oldest/dim */
```

**Effect:** In RAINBOW mode the particle head points in one of 8 directions and its color matches its direction — a rotating color wheel where the hue literally tells you where the flow is going. In mono mode the trail fades from bright to dim as it ages, encoding time as color.

---

## 14. Normal-to-Luminance Surface Shading

**Where:** sphere_raster.c, cube_raster.c, torus_raster.c, displace_raster.c

**How it works:**
The rasterizer pipeline maps 3D surface normals to displayed characters and colors through a three-step chain:

1. **Normal → luminance via dot product:** `luma = max(0, dot(N, L))` (Lambertian diffuse, where `L` is the normalised light direction). Optional Blinn-Phong specular `(max(0, dot(N, H))^shininess)` is added for highlights.
2. **Luminance → gamma correction:** `luma_corrected = pow(luma, 1/2.2)` maps linear light to the perceptually uniform display value.
3. **Corrected luminance → character + color pair (dual axis):** The same float drives two independent outputs:
   - `idx = (int)(luma * BOURKE_LEN)` → `k_bourke[idx]` — character density encodes brightness
   - `cp = 1 + (int)(luma * 6)` → warm-to-cool pair (1=red → 7=magenta) — color hue encodes brightness

```c
/* luma_to_cell() in torus_raster.c */
float d = luma + (k_bayer[py & 3][px & 3] - 0.5f) * 0.15f;   /* Bayer dither */
d = fmaxf(0.0f, fminf(1.0f, d));
int  idx  = (int)(d * (BOURKE_LEN - 1));
char ch   = k_bourke[idx];
int  cp   = 1 + (int)(d * 6.0f);                               /* 1–7 warm→cool */
bool bold = d > 0.6f;
return (Cell){ ch, cp, bold };
```

**Effect:** A 3D surface gets two independent brightness signals per cell — one from the character's ink density and one from its color hue — giving approximately 700 distinguishable brightness levels in a single terminal cell. Lit areas appear warm/yellowish; shadow areas appear cool/blueish; matching the color temperature of real light sources.

---

## 15. Per-Primitive Material Colors

**Where:** raymarcher/raymarcher_primitives.c

**How it works:**
The sphere-marching loop tracks which SDF primitive returned the closest distance at the current hit point. Each primitive is assigned a material ID; the ID maps to a distinct grey-ramp color pair. This lets multiple objects in the scene render at different luminance levels using only the existing 8-pair grey ramp — no additional color registration is needed.

```c
/* map() returns both distance and material ID */
if (d_sphere < min_d) { min_d = d_sphere; mat_id = MAT_SPHERE; }
if (d_box    < min_d) { min_d = d_box;    mat_id = MAT_BOX;    }
/* ... */

/* cbuf stores the material → fb_blit uses it as pair index */
cbuf[idx] = (Cell){ ch, mat_id_to_pair[mat_id], bold };
```

**Material → pair mapping:**
| Material | Pair | Grey shade | Appearance |
|----------|------|-----------|------------|
| MAT_SPHERE | 8 | 255 (white) | Brightest |
| MAT_BOX | 6 | 250 | Mid-light |
| MAT_TORUS | 4 | 244 | Mid |
| MAT_CAPSULE | 2 | 238 | Dark |
| MAT_CONE | 1 | 235 | Darkest |

**Effect:** Each SDF primitive renders in a visually distinct shade, making the composition legible at a glance without any mesh or UV system — just a one-integer material tag per ray hit.

---

## 16. Color Pair Allocation Reference

**How it works:**
ncurses reserves pair 0 as the default (unmodifiable), giving 255 usable pairs (1–255 on most terminals; `COLOR_PAIRS` at runtime). None of the animations in this project approach this limit. Pairs are allocated contiguously from 1.

**Pair counts per animation:**

| File | Pairs allocated | Purpose |
|------|----------------|---------|
| fire.c | 9 (+ theme re-use) | 9-level heat ramp, re-registered per theme |
| aafire_port.c | 9 | Same as fire.c |
| matrix_rain.c | 6 | 6 shade levels (FADE→HEAD) |
| flocking.c | 3 | One per flock, re-registered every N frames |
| constellation.c | 7 | Spectral hues |
| fireworks.c | 7 | Spectral hues |
| brust.c | 7 | Spectral hues |
| kaboom.c | 7 | Role-named pairs (blob far/mid/near, flash, ring, HUD) |
| donut.c | 8 | Grey ramp 235–255 |
| raymarcher*.c | 8 | Grey ramp 235–255 |
| torus/cube/sphere/displace_raster.c | 7 | Warm-to-cool luminance |
| flowfield.c | 8 | 8 hue/age levels |
| sand.c | 4 | Grain density levels |
| bonsai.c | 6 | Brown trunk, branches, leaf shades |
| snowflake.c | 7 | 6 ice gradient pairs + 1 walker pair |
| coral.c | 7 | 6 vivid coral pairs + 1 walker pair |
| sierpinski.c | 4 | 3 vertex colors + 1 HUD |
| fern.c | 6 | 5 green-gradient transform colors + 1 HUD |
| julia.c | 6 | Fire palette: white→yellow→orange→red + HUD |
| mandelbrot.c | 6 | Neon palette: magenta→yellow→lime→cyan→purple + HUD |
| koch.c | 6 | 5-color vivid gradient (cyan→teal→lime→yellow→white) + HUD |
| lightning.c | 5 | 3 depth bands + 2 glow/halo pairs |
| wave_interference.c | 8 | CP_N3..CP_P3 signed ramp + CP_HUD |
| led_number_morph.c | 12 | CP_D0..CP_D9 digit hues + CP_HUD + CP_OFF |
| particle_number_morph.c | 12 | CP_D0..CP_D9 digit hues + CP_HUD + CP_IDLE |
| julia_explorer.c | 7 | 5 escape-time bands + CP_HUD + CP_XH crosshair |
| barnsley.c | 6 | 4 log-density levels + CP_HUD + CP_WALK |
| diffusion_map.c | 7 | 5 age-gradient levels + CP_HUD + CP_WALK |
| tree_la.c | 7 | 5 phi-gradient levels + CP_HUD + CP_WALK |
| lyapunov.c | 8 | CP_BG + CP_S1..S3 (blue stable) + CP_C1..C3 (red chaotic) + CP_HUD |
| barnes_hut.c | 8 | CP_HUD + CP_L1..CP_L5 glow/speed + CP_TREE + CP_BH |

**Dynamic re-registration:** Several animations call `init_pair()` mid-loop. This is safe — ncurses applies the new color on the next `doupdate()`. The pair number itself does not change, only the color behind it. There is no risk of "stale" pair handles.

**8-color terminal note:** When `COLORS < 256`, xterm-256 indices are not available. All files use the `if (COLORS >= 256)` fallback that registers standard `COLOR_*` constants instead. The pair numbers stay the same; only the registered color differs.

---

## 18. Distance-based Coloring (Snowflake)

**Where:** snowflake.c

**How it works:**
At freeze time, the Euclidean distance from the crystal centre to the new cell is computed. This distance is divided into bands that map to six color pairs, forming a gradient from inner core to outer tips:

```c
float frac = dist / g->max_dist;   /* 0 = centre, 1 = outermost tip */

if      (frac < 0.15f) color = COL_ICE_6;  /* 117 light blue  — core  */
else if (frac < 0.30f) color = COL_ICE_5;  /* 38  ocean blue           */
else if (frac < 0.50f) color = COL_ICE_4;  /* 44  medium teal          */
else if (frac < 0.65f) color = COL_ICE_3;  /* 51  bright cyan          */
else if (frac < 0.80f) color = COL_ICE_2;  /* 195 pale ice             */
else                   color = COL_ICE_1;  /* 231 white — tips         */
```

`max_dist` is updated incrementally as new cells freeze, so the color scale stretches as the crystal grows: inner cells are remapped to progressively lighter colors as outer cells push `max_dist` larger.

**Effect:** The crystal reads as a physical object — cold blue at the dense core, icy white at the sharp outer tips. Color conveys spatial depth without any 3-D computation.

---

## 19. Escape-time Band Coloring (Julia / Mandelbrot)

**Where:** julia.c, mandelbrot.c

**How it works:**
The escape-time iteration runs until `|z|² > 4` or `MAX_ITER` is reached. The fractional escape ratio `frac = iter / MAX_ITER` is mapped to discrete color bands:

```c
static ColorID escape_color(int iter, int max_iter) {
    float frac = (float)iter / (float)max_iter;
    if (frac < THRESHOLD)  return COL_BG;    /* inside set, background */
    if (frac < 0.30f)      return COL_C2;    /* slow escape — bright   */
    if (frac < 0.55f)      return COL_C3;
    if (frac < 0.75f)      return COL_C4;
    return                        COL_C5;    /* fast escape — dim      */
}
```

Pixels that escape slowly are those near the boundary of the set — they receive the most vivid colors. Pixels deep inside the set never escape and remain the background color. Pixels that escape quickly (far outside) get the dimmest colors, forming the dark outer halo.

`julia.c` uses a fire palette (white→yellow→orange→red); `mandelbrot.c` uses an electric neon palette (magenta→yellow→lime→cyan→purple). The two files keep the same `escape_color()` logic but register different xterm-256 indices for COL_C2..C5.

**Effect:** The fractal boundary is highlighted by the densest color bands. The image reads "hot at the edge, cold inside" (julia fire) or "electric boundary on dark space" (mandelbrot neon).

---

## 20. IFS Vertex Coloring (Sierpinski / Fern)

**Where:** sierpinski.c, fern.c

**How it works:**
In the chaos game, every iteration picks one transform from the IFS table. The index of the chosen transform is used directly as the plot color. For the Sierpinski triangle (3 transforms → 3 colors):

```c
int v = rand() % 3;            /* chosen vertex 0, 1, or 2 */
*which = v;
/* ... caller: */
grid_plot(&grid, fx, fy, (uint8_t)(which + 1));  /* color pair 1, 2, or 3 */
```

Because each transform maps the attractor to a specific sub-region, the color partitions the fractal into self-similar colored sub-triangles. The bottom-left sub-triangle is always cyan, bottom-right is yellow, top is magenta — and the same three-color pattern repeats at every scale.

For `fern.c` the color is fixed per transform (stem=green, main=bright-green, left=lime, right=yellow-lime). The coloring reveals which transform generated each point, making the fern's self-similar structure visible.

**Effect:** IFS vertex coloring reveals the algebraic structure of the fractal — the sub-regions generated by each transform are visually separated, giving insight into how the IFS builds the attractor.

---

## 21. Depth-position Coloring (Koch / Lightning)

**Where:** koch.c, lightning.c

**How it works:**
Color is assigned based on the index or position of the element being drawn, creating a gradient across the structure.

In `koch.c`, each line segment receives a color from a gradient palette indexed by `seg_draw_index / n_segs`:

```c
int ci = (int)((float)seg_idx / (float)n_segs * (N_COLORS - 1));
attr_t attr = COLOR_PAIR(color_ids[ci]) | A_BOLD;
```

This makes early-drawn segments (the original triangle edges) one color and late-drawn segments (the fine tips) another color — producing a radial gradient from inside to outside.

In `lightning.c`, color is assigned by the row of each frozen cell:

```c
int top_third    = g->rows / 3;
int bottom_third = 2 * g->rows / 3;
if      (row < top_third)    color = COL_BOLT_TOP;    /* xterm 45 light blue */
else if (row < bottom_third) color = COL_BOLT_MID;    /* xterm 51 teal       */
else                         color = COL_BOLT_BOT;    /* xterm 231 white     */
```

**Effect:** The lightning bolt reads as a physical discharge — blue at the cloud, whitening toward the ground where energy is dissipated. The Koch snowflake reads as an architectural structure — base in one hue, fine detail in another.

---

## 22. Density-Accumulator Coloring (Buddhabrot)

**Where:** buddhabrot.c

**How it works:**
The Buddhabrot builds a `uint32_t counts[rows][cols]` hit-count grid. Every orbit point increments its cell's counter. At draw time, each non-zero cell is mapped to one of five color bands using log-normalized density.

**Log normalization:**
```c
float t = logf(1.0f + (float)count)
        / logf(1.0f + (float)max_count);
```
`log(1+x)` compresses the extreme dynamic range: anti-mode attractor cells accumulate millions of hits while transient cells get 1–5. With sqrt normalization, `sqrt(1/10⁶) ≈ 0.001` still falls in the lowest visible band — producing scattered dots across a blank screen. With log, the same cell maps to ≈0.035 — below the invisible floor.

**Mode-aware invisible floor:**
```c
float floor = anti ? 0.25f : 0.05f;
if (t < floor) return 0;   /* draw nothing */
```
Anti-mode uses a high floor (0.25) because its max_count is enormous. Buddha mode uses a low floor (0.05) to keep fine orbital structure visible.

**Five-level nebula palette** — purple→white gradient maps orbital density to perceived luminosity:

| Band | t range | xterm index | Color | Char |
|------|---------|-------------|-------|------|
| C1 | 0.05–0.45 (buddha) / 0.25–0.45 (anti) | 55 | Dark blue-purple | `.` |
| C2 | 0.45–0.62 | 93 | Violet | `:` |
| C3 | 0.62–0.78 | 141 | Light purple | `+` |
| C4 | 0.78–0.90 | 183 | Lavender-pink | `#` |
| C5 | ≥ 0.90 | 231 | White (bold) | `@` |

**Effect:** The completed image resembles a luminous nebula — sparse orbital filaments in dark purple, dense orbital crossings in white. Anti-mode produces a bright interior attractor surrounded by radiating structure.

---

## 17. xterm-256 Palette Index Reference

**How it works:**
The xterm-256 color space has three regions:
- **0–15:** Standard and high-intensity named colors
- **16–231:** 6×6×6 RGB cube — index = `16 + 36*r + 6*g + b` where r,g,b ∈ [0,5]
- **232–255:** 24-step grey ramp from near-black to near-white

Specific indices are chosen for maximum visibility on black backgrounds.

**Commonly used indices in this project:**
| Index | Color            | Used in                              |
|-------|------------------|--------------------------------------|
| 196   | Bright red       | fire, fireworks                      |
| 202   | Fire orange      | fire themes                          |
| 214   | Amber            | fire, gold theme                     |
| 226   | Bright yellow    | fire, gold theme; julia, mandelbrot  |
| 208   | Orange           | julia fire palette                   |
| 124   | Dark red         | julia fire palette (outer)           |
| 231   | White            | julia bright interior; lightning bot |
| 201   | Hot magenta      | plasma theme; mandelbrot inside      |
| 141   | Light purple     | mandelbrot outer band                |
| 82    | Lime green       | mandelbrot band                      |
| 118   | Bright lime      | fern; koch; coral                    |
| 154   | Yellow-lime      | fern tip; coral                      |
| 46    | Matrix green     | nova theme, matrix                   |
| 51    | Bright cyan      | ice theme; koch; snowflake mid       |
| 87    | Electric cyan    | sierpinski vertex 1                  |
| 207   | Hot magenta      | sierpinski vertex 3                  |
| 117   | Light blue       | snowflake core                       |
| 195   | Pale ice         | snowflake mid-outer                  |
| 44    | Medium teal      | snowflake teal band                  |
| 38    | Ocean blue       | snowflake inner-mid                  |
| 45    | Sky blue         | lightning top; snowflake walker      |
| 33    | Dodger blue      | flocking                             |
| 55    | Dark blue-purple | buddhabrot band 1 (lowest density)   |
| 93    | Violet           | buddhabrot band 2                    |
| 183   | Lavender-pink    | buddhabrot band 4                    |
| 213   | Pink-magenta     | bat group 2                          |
| 203   | Coral red        | coral aggregate layer 1              |
| 86    | Teal-cyan        | coral; koch level 2                  |
| 232   | Near-black       | theme backgrounds                    |
| 235–255 | Grey ramp steps | donut, rasters                    |

**Effect:** Predictable, terminal-portable colors without relying on named color constants that vary by terminal theme.

---

## 23. Signed 8-Level Color Ramp (Wave Interference)

**Where:** fluid/wave_interference.c

**How it works:**
The superposition of N waves produces a value in `[-N, +N]`. This signed range is mapped to 8 color pairs: 3 for negative pressure (blue cold), 1 neutral, and 3 for positive pressure (red hot), plus a white peak.

```c
enum { CP_N3=1, CP_N2, CP_N1, CP_Z, CP_P1, CP_P2, CP_P3, CP_HUD };

/* Map normalised value in [-1,+1] to pair */
int val_to_pair(float v) {
    if      (v < -0.60f) return CP_N3;
    else if (v < -0.20f) return CP_N2;
    else if (v < -0.02f) return CP_N1;
    else if (v <  0.02f) return CP_Z;
    else if (v <  0.20f) return CP_P1;
    else if (v <  0.60f) return CP_P2;
    else                 return CP_P3;
}
```

Phase is precomputed once per source per cell: `g_phase[s][r][c] = k * dist(source, cell)` where `k = 2π/wavelength`. Per frame: `sum = Σ cos(g_phase[s][r][c] - omega*t)`, normalised by N sources, mapped to the 8-level ramp.

**Effect:** Constructive interference nodes glow red/white; destructive nodes glow deep blue. The standing wave pattern is immediately legible from color alone.

---

## 24. Log-Density Accumulator Coloring (Barnsley / DLA)

**Where:** fractal_random/barnsley.c, fractal_random/diffusion_map.c, fractal_random/tree_la.c

**How it works:**
Each cell accumulates an integer hit count. Log-normalisation compresses the extreme dynamic range (attractor hot-spots receive millions of hits while transient cells receive 1–5):

```c
float t = logf(1.0f + (float)count)
        / logf(1.0f + (float)max_count);
```

The normalised value maps to 4 character levels: `. : + @` (density ramp), each assigned a distinct color pair.

For diffusion_map.c and tree_la.c, age (in ticks since a cell was added) replaces hit count — old cells are bright, new cells are dim — creating a gradient that shows growth history.

**Effect:** Log normalization makes both rare filaments and dense cores visible simultaneously. Without it, the vast range in hit counts forces a binary (present/absent) rendering.

---

## 25. Lyapunov Fractal — Dual-Palette Signed Value

**Where:** fractal_random/lyapunov.c

**How it works:**
Each pixel is a point `(a, b)` in parameter space of the logistic map. The Lyapunov exponent `λ = (1/N)·Σ ln|r(1−2xₙ)|` is computed; its sign determines stability.

Two separate 3-level palettes are used for the two sign cases:

```c
/* λ < 0 → stable: blue gradient */
enum { CP_BG, CP_S1, CP_S2, CP_S3, CP_C1, CP_C2, CP_C3, CP_HUD };

float mag = fabsf(lambda) / lambda_max;
int lvl = (int)(mag * 2.9f);   /* 0..2 */
int pair = (lambda < 0) ? (CP_S1 + lvl) : (CP_C1 + lvl);
```

Stable regions (λ < 0): dark→medium→bright blue. Chaotic regions (λ > 0): dark→medium→bright red. The boundary `λ = 0` appears as the exact color transition.

**Effect:** The Lyapunov fractal reads as a map of order vs chaos. Blue = the logistic map converges to a fixed point or cycle. Red = it diverges into chaotic behaviour. The boundary is the fractal.

---

## 26. Glow Accumulator + Speed-Normalized Body Coloring (Barnes-Hut)

**Where:** physics/barnes_hut.c

**How it works:**
Two layers are rendered per frame:

**Layer 1 — Glow accumulator.** Each frame every active body increments `g_bright[row][col]`. The entire grid decays by `DECAY = 0.93` per render frame. This creates orbital trails that persist for ~60 frames.

```c
g_bright[cr][cc] += 1.0f;           /* accumulate */
g_bright[r][c]   *= DECAY;          /* decay every render */
/* render " . : o O @ " ramp from normalised brightness */
```

**Layer 2 — Direct body glyphs.** After the glow layer, every active body is drawn at its current pixel position with a character and color determined by its speed relative to the rolling maximum:

```c
float norm = spd / g_v_max;         /* 0=still, 1=fastest body */

if      (norm > 0.80f) { pair = CP_L5; ch = '*'; }  /* blazing fast */
else if (norm > 0.55f) { pair = CP_L4; ch = '+'; }
else if (norm > 0.30f) { pair = CP_L3; ch = 'o'; }
else if (norm > 0.10f) { pair = CP_L2; ch = '.'; }
else                   { pair = CP_L1; ch = ','; }  /* nearly still */
```

`g_v_max` decays by 0.9995× per tick so the scale adapts to current dynamics — after a burst of ejections the scale re-anchors to the surviving population.

**Effect:** Glow layer shows orbital structure; direct glyphs ensure bodies are always visible even when moving fast. Speed coloring reveals velocity gradients — Keplerian disk shows cold outer bodies, hot fast inner bodies.

---

## 27. Encapsulated Theme Struct with Special-Role Pairs

**Where:** physics/barnes_hut.c, fluid/wave_interference.c, artistic/led_number_morph.c

**How it works:**
When a simulation needs both a 5-level ramp AND one or more semantically distinct pairs (e.g. quadtree overlay, central body highlight, HUD), the theme struct stores all of them:

```c
typedef struct {
    const char *name;
    int hi256[5];    /* CP_L1..CP_L5 in 256-color mode */
    int hi8[5];      /* CP_L1..CP_L5 in 8-color mode */
    int tree256;     /* CP_TREE — quadtree grid lines */
    int tree8;
    int bh256;       /* CP_BH  — central black hole glyph */
    int bh8;
} Theme;

static void color_init(int theme) {
    const Theme *th = &k_themes[theme];
    for (int i = 0; i < 5; i++)
        init_pair(CP_L1 + i, th->hi256[i], -1);
    init_pair(CP_TREE, th->tree256, -1);
    init_pair(CP_BH,   th->bh256,  -1);
}
```

Switching themes calls `color_init(new_theme)` which re-registers all pairs — the pair numbers never change, only the registered colors. This means `COLOR_PAIR(CP_BH)` keeps working; the terminal just sees a new color behind the same handle.

**Effect:** All color semantics are concentrated in one struct per theme. Adding a new theme is one line in the `k_themes` array. The rendering code is color-agnostic.

---

## 16. Bright Hue-varying Theme Palette (Raymarcher)

**Where:** `raymarcher/sdf_gallery.c`

**Problem:** A palette that ramps from dark to bright within one hue has invisible low-gradient steps on a dark terminal background. When the scene's `col` field maps to those steps, parts of the geometry appear unrendered.

**Solution:** Keep every palette entry at full saturation — vary **hue** across the gradient instead of brightness.

```c
/* Bad: dark-to-bright in one hue — low steps invisible on dark bg */
{17, 18, 19, 20, 27, 33, 45, 159}   /* indices 0-3 are near-black */

/* Good: hue varies, all steps vivid */
{51, 87, 123, 159, 153, 189, 225, 231}  /* cyan→sky→lavender→white */
```

**Verification rule:** decode any xterm-256 index to RGB components:
```
r = (color - 16) / 36
g = ((color - 16) % 36) / 6
b = (color - 16) % 6
```
A color is "bright enough" when `max(r, g, b) >= 4`. Entries with `max < 4` will appear
dark or invisible against a black background.

**xterm-256 bright reference points:**
| Index | RGB (0-5) | Appearance |
|-------|-----------|------------|
| 46  | (0,5,0) | saturated green |
| 51  | (0,5,5) | saturated cyan |
| 196 | (5,0,0) | saturated red |
| 201 | (5,0,5) | saturated magenta |
| 214 | (5,3,0) | bright orange |
| 226 | (5,5,0) | bright yellow |
| 231 | (5,5,5) | white |

**Effect:** Every gradient step is readable at all times. Theme identity comes from
hue family (fire=red/orange, arctic=cyan/blue) rather than the dark-end color,
which was invisible anyway.
