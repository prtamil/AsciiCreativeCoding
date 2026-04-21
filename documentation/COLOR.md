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

**Where:** fire.c, smoke.c, aafire_port.c

**How it works:**
Each theme struct contains 9 xterm-256 color indices covering a gradient from cold/sparse to hot/dense. Themes switch on manual `t` keypress. `theme_apply()` re-registers all color pairs with the new theme's indices — no visual structure changes, only the palette. This pattern is shared by fire.c (6 fire themes), smoke.c (6 smoke themes), and aafire_port.c.

**fire.c themes:**
| Name   | Colors                                |
|--------|---------------------------------------|
| fire   | Red → orange → yellow                 |
| ice    | Sky blue → cyan → white               |
| plasma | Violet → magenta → white              |
| nova   | Green → lime → white                  |
| poison | Olive → yellow-green → white          |
| gold   | Amber → orange → yellow               |

**smoke.c themes:**
| Name   | Colors                                |
|--------|---------------------------------------|
| gray   | Medium gray → white                   |
| soot   | Cool dark-gray → light-gray           |
| steam  | Sky blue → cyan → white               |
| toxic  | Mid green → bright lime               |
| ember  | Orange → yellow → white               |
| arcane | Violet → pink → white                 |

All palettes start at clearly visible mid-tones — no near-black (232–236) entries so even the faintest wisps show up on dark terminals.

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

**Where:** `flowfield.c`, `complex_flowfield.c`

**How it works — flowfield.c (8 pairs, 4 fixed themes):**
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

## 13b. Pre-Baked Cosine Palette — Angle-to-Color (complex_flowfield.c)

**Where:** `complex_flowfield.c`

**How it works:**
`complex_flowfield.c` replaces the fixed 8-color per-theme arrays with a cosine palette pre-baked into 16 color pairs at theme-change time. The cosine formula `color(t) = a + b·cos(2π·(c·t + d))` is evaluated at 16 evenly-spaced `t` values and each result is mapped to an xterm-256 cube index:

```c
/* Pre-bake 16 pairs for the active theme */
for (int i = 0; i < 16; i++) {
    float t  = (float)i / 15.f;          /* t ∈ [0, 1] */
    float rf = a[0] + b[0] * cosf(2.f*M_PI*(c[0]*t + d[0]));  /* clamped */
    float gf = a[1] + b[1] * cosf(2.f*M_PI*(c[1]*t + d[1]));
    float bf = a[2] + b[2] * cosf(2.f*M_PI*(c[2]*t + d[2]));
    int fg = 16 + 36*(int)(rf*5+.5f) + 6*(int)(gf*5+.5f) + (int)(bf*5+.5f);
    init_pair(CP_BASE + i, fg, COLOR_BLACK);
}

/* Runtime: normalize angle to t, select pair */
float t    = (angle < 0.f ? angle + 2.f*M_PI : angle) / (2.f*M_PI);
int   pair = CP_BASE + (int)(t * 16) % 16;
```

Six themes are defined as `CosTheme { a[3], b[3], c[3], d[3], name }` structs — **cosmic** (violet→cyan→magenta), **ember** (red→orange→gold), **ocean** (navy→teal→cyan), **neon** (green→pink→violet), **sunset** (purple→crimson→amber), **mono** (silver-blue gradient).

**Key difference from flocking.c:** `flocking.c` calls `init_pair()` every N animation frames to continuously animate the palette (real-time color cycling). `complex_flowfield.c` calls it once per theme change only — the palette is static between theme changes. The angle-to-pair mapping means adjacent flow directions always get adjacent palette colors, so the full-screen colormap (bg_mode 2) shows a smooth hue gradient that exactly mirrors the flow topology.

**Effect:** In colormap mode (`v` key), every terminal cell is colored by its local flow angle via the cosine palette, producing a full-screen procedurally generated image. Switching field type or theme completely transforms the picture. Particle trails drawn on top at `A_BOLD` remain visible over the colormap background.

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
| complex_flowfield.c | 16 | 16-step cosine palette, pre-baked at theme change; angle→pair at runtime |
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

---

## 28. Root-Basin Coloring with Convergence-Speed Brightness (Newton Fractal)

**Where:** `fractal_random/newton_fractal.c`

**How it works:**
Newton's method for `f(z) = z⁴ − 1` converges to one of four roots: +1, −1, +i, −i. Each root is assigned a hue family (red, blue, yellow, green). Within each hue, two color pairs are used: one dark and one bright. The convergence speed `t = n_iter / MAX_ITER` selects which:

```c
/* two pairs per root: dark (D) and bright (B) */
enum {
    CP_R1D = 1,  /* root +1  dark red    */
    CP_R1B,      /* root +1  bright red  */
    CP_R2D,      /* root -1  dark blue   */
    CP_R2B,      /* root -1  bright blue */
    /* ... */
};

static void map_color(int root, int n, int *cp_out, chtype *ch_out) {
    float t = (float)n / (float)MAX_ITER;
    int bright = (t < 0.25f);           /* fast convergence = bright */
    *cp_out = CP_R1D + root * 2 + (bright ? 1 : 0);

    if      (t < 0.15f) *ch_out = '.';  /* slowest converge near boundary */
    else if (t < 0.40f) *ch_out = ':';
    else if (t < 0.70f) *ch_out = '+';
    else                *ch_out = '#';  /* fastest converge = dense char   */
}
```

The inversion relationship: fast convergence (low `t`) maps to bright colors and sparse characters (`.`), while slow convergence near basin boundaries maps to dark colors and dense characters (`#`). This makes the fractal boundary — where convergence is most uncertain — appear as a vivid dark-outlined fractal structure.

**Effect:** Four-color basin map with internal shading. The basin interiors show gradient structure (where Newton converges in fewer steps) while the boundary fractal is highlighted by the darkest colors and densest characters.

---

## 29. Continuous-State CA — 5-Level Color Quantization (Lenia)

**Where:** `fluid/lenia.c`

**How it works:**
The Lenia automaton produces a continuous float state `u ∈ [0, 1]` per cell. Rather than mapping this to a grey ramp, it is quantized into 5 blue-spectrum pairs spanning dark navy to white, paired with increasing-density characters. Cells with `u < 0.05` are skipped entirely (transparent background), keeping the display sparse and readable:

```c
if      (u < 0.05f) continue;          /* invisible — background shows */
else if (u < 0.20f) { cp = CP_U0; ch = '.'; }   /* dark navy   */
else if (u < 0.40f) { cp = CP_U1; ch = ':'; }   /* blue        */
else if (u < 0.60f) { cp = CP_U2; ch = '+'; }   /* cyan        */
else if (u < 0.80f) { cp = CP_U3; ch = '*'; }   /* pale cyan   */
else                { cp = CP_U4; ch = '#'; }   /* white       */

/* Color pairs: navy→blue→cyan→pale-cyan→white */
init_pair(CP_U0,  17, -1);   /* dark navy  */
init_pair(CP_U1,  27, -1);   /* blue       */
init_pair(CP_U2,  51, -1);   /* cyan       */
init_pair(CP_U3, 195, -1);   /* pale cyan  */
init_pair(CP_U4, 231, -1);   /* white      */
```

Both character density and color advance together through the same threshold boundaries — `.` at the faintest and `#` at the brightest — giving two independent visual channels that reinforce each other.

**Effect:** Living "creatures" appear as bright white-cyan cores fading to dark navy edges over a transparent background, matching the bioluminescent aesthetic of the Lenia biology paper.

---

## 30. Multi-Component Physics Coloring — Role-Semantic Pair Assignment (Schrödinger)

**Where:** `physics/schrodinger.c`

**How it works:**
When a simulation has several physically distinct quantities that must be visually separable, each quantity gets its own color pair with a role name rather than a numeric index. For the 1-D Schrödinger equation, four quantities are displayed simultaneously:

```c
enum { CP_RE=1, CP_IM, CP_PROB, CP_VPOT, CP_HUD };

init_pair(CP_RE,   27, -1);   /* blue   — Re(ψ)  real part      */
init_pair(CP_IM,  196, -1);   /* red    — Im(ψ)  imaginary part */
init_pair(CP_PROB,231, -1);   /* white  — |ψ|²   probability density */
init_pair(CP_VPOT,220, -1);   /* yellow — V(x)   potential energy    */
```

The rendering draws all four in a single column pass:
- `|ψ|²` as a vertical white bar chart growing upward from the midline
- `Re(ψ)` as a blue dot at the signed-amplitude row
- `Im(ψ)` as a red dot at its signed-amplitude row
- `V(x)` as a yellow bar at the bottom

Because `Re`, `Im`, and `|ψ|²` are drawn at different rows for the same column, they do not overwrite each other — the last write at each cell wins, so `|ψ|²` is drawn first (background) and `Re`/`Im` dots are drawn on top.

**Effect:** All three wavefunction components are simultaneously visible on screen. Blue and red oscillate together (the complex phasor rotation); white shows the probability envelope; yellow shows the potential barrier, all without any intermediate buffer or separate window.

---

## 31. Iso-Level Hue Shift — Progressive Color Stepping Per Contour (Marching Squares)

**Where:** `fluid/marching_squares.c`

**How it works:**
In multi-level mode, the marching squares algorithm draws N iso-contours at evenly spaced threshold values. Rather than reusing the same color pair for all levels, each level's color pair is dynamically registered with an xterm-256 index offset from the theme base color. This produces a spectrum of contours with distinct hues without pre-allocating N theme-specific pairs:

```c
/* evenly spaced levels from thresh*0.3 up to thresh*0.95 */
float t = a->thresh * (0.3f + lv * 0.15f);

/* dynamic hue shift: each level gets +6 index steps */
short col = (short)(theme_contour[a->theme] + lv * 6);
init_pair(10 + (short)lv, col, 16);   /* pairs 10..14 overwritten each frame */
attron(COLOR_PAIR(10 + lv));
mvaddch(gy, gx, c);
attroff(COLOR_PAIR(10 + lv));
```

The base theme indices are chosen at saturated, well-spaced xterm-256 positions (51 cyan, 196 red, 46 green, 201 magenta). Adding `lv * 6` to the base index steps through nearby hues in the 6×6×6 color cube, keeping the theme family while varying the level.

**Effect:** Each iso-contour ring appears in a slightly different hue, creating a topographic-map look that reveals the depth structure of the potential field. The fill inside the innermost contour is drawn with a faint backtick character in `CP_INSIDE`, giving density without clutter.

---

## 32. Particle-Type Color Struct — Physics Properties Co-Located with Color (Bubble Chamber)

**Where:** `physics/bubble_chamber.c`

**How it works:**
Rather than separately managing physics parameters and color indices, each particle type is defined as a single struct that co-locates the charge/mass ratio, display symbol, and both 256-color and 8-color fallback indices. Color pairs are registered in a loop directly from this table:

```c
typedef struct {
    const char *name;
    const char *symbol;
    float       qm;       /* charge/mass ratio — determines curl direction */
    short       c256;     /* xterm-256 foreground                          */
    short       c8;       /* 8-color fallback                              */
} PType;

static const PType k_types[N_TYPES] = {
    { "electron", "e-", -0.200f,  51, COLOR_BLUE   },   /* tight cyan spirals  */
    { "positron", "e+", +0.200f, 196, COLOR_RED    },   /* tight red spirals   */
    { "muon",     "mu", -0.070f,  46, COLOR_GREEN  },   /* medium green arc    */
    { "pion",     "pi", +0.045f, 226, COLOR_YELLOW },   /* wide yellow arc     */
    { "proton",   "p ", +0.022f, 159, COLOR_CYAN   },   /* barely curves, cyan */
};

for (int i = 0; i < N_TYPES; i++) {
    short fg = g_256color ? k_types[i].c256 : k_types[i].c8;
    init_pair(1 + i, fg, -1);
}
```

The color then directly encodes physics: opposite `qm` signs produce opposite curl directions, and the color distinguishes electron (cyan, tight) from positron (red, tight) and proton (pale cyan, almost straight).

**Trail age fading** uses the same pair but with attribute stepping — `A_BOLD` for fresh track, normal for mid-age, and the track is simply omitted for `age >= 0.80`:

```c
if      (age < 0.25f) { ch = '*'; attr |= A_BOLD; }
else if (age < 0.55f) { ch = '+'; }
else                  { ch = '.'; }
/* age >= 0.80: break — rest of trail not drawn */
```

**Effect:** Each particle type is immediately identifiable by color and curl direction. Trail age is encoded in character density without additional pair registration.

---

## 33. Recursive Depth Rainbow — Hue Encodes Fractal Nesting Level (Apollonian Gasket)

**Where:** `fractal_random/apollonian.c`

**How it works:**
Each circle in the Apollonian gasket is generated at a recursion depth 1–7. The color pair is assigned directly from the depth, cycling through a full hue spectrum from yellow (outermost, depth 1) to red (deepest, depth 7):

```c
init_pair(CP_D1, 226, -1);   /* bright yellow — depth 1 (outermost) */
init_pair(CP_D2, 118, -1);   /* lime green    — depth 2             */
init_pair(CP_D3,  51, -1);   /* bright cyan   — depth 3             */
init_pair(CP_D4,  33, -1);   /* dodger blue   — depth 4             */
init_pair(CP_D5,  93, -1);   /* purple        — depth 5             */
init_pair(CP_D6, 201, -1);   /* magenta       — depth 6             */
init_pair(CP_D7, 196, -1);   /* red           — depth 7 (deepest)   */

/* At draw time: clamp depth to [1,7] and select bold for outer rings */
int d    = c->depth < 1 ? 1 : c->depth > 7 ? 7 : c->depth;
attr_t bold = (d <= 2) ? A_BOLD : 0;   /* larger outer circles bolder */
attron(COLOR_PAIR(d) | bold);
```

Sub-pixel circles (radius < 0.5 cols) are drawn as a single `.` dot; tiny circles (radius < 1.5 cols) as `o` or `.` depending on depth; full circles draw their outline using slope characters. This avoids spending rendering budget on circles too small to show their outline.

**Effect:** The fractal's self-similar nesting structure is immediately legible — the same color sequence (yellow→green→cyan→blue→purple→magenta→red) repeats at every zoom level, revealing the recursive construction.

---

## 34. Doppler + Gravitational Redshift Color (Black Hole Accretion Disk)

**Where:** `artistic/blackhole.c`

**How it works:**
The brightness of each disk pixel is computed from three multiplicative physical factors: relativistic Doppler beaming, gravitational redshift, and a radial temperature profile. This combined brightness then drives both the character selected and the color pair:

```c
/* Keplerian Doppler beaming: approaching side → D > 1 (brighter) */
float v_orb = sqrtf(0.5f / disk_r);
float beta  = -v_orb * cosf(phi) * cos_tilt;
float D     = powf((1.f+beta)/(1.f-beta), 1.5f);

/* Gravitational redshift: inner disk dims near horizon */
float g = sqrtf(fmaxf(0.01f, 1.f - 1.f / disk_r));

/* Radial temperature + ISCO spike */
float dr   = disk_r - DISK_IN;
float isco = expf(-dr * dr * 0.65f);   /* spike at innermost stable orbit */
float rad  = powf(1.f - 0.86f * r_n, 2.2f) + 0.65f * isco;

/* Spiral texture co-rotating with disk */
float tex = 1.f + 0.18f * sinf(disk_phi * 5.f - disk_angle * 4.f);

float bright = clamp(D * g * rad * tex, 0.f, 1.f);

/* bright → color pair (inner=hot, outer=cool) */
if      (bright > 0.85f) { cp = CP_RING; a = A_BOLD; }
else if (bright > 0.67f) { cp = CP_HOT;              }
else if (bright > 0.50f) { cp = CP_WARM;             }
else if (bright > 0.33f) { cp = CP_MID;              }
else if (bright > 0.17f) { cp = CP_COOL;             }
else                     { cp = CP_DIM;              }
```

The theme system supports 11 named palettes (interstellar, matrix, nova, ocean, etc.), each defining the six foreground colors for CP_RING through CP_DIM. Switching themes re-registers all six pairs, keeping the same brightness logic but changing the color family.

**Effect:** The approaching side of the accretion disk is dramatically brighter (Doppler boost up to 4×), gravitational redshift dims the innermost disk, and the ISCO spike creates a bright ring at the innermost stable orbit. All visible on a character terminal with no GPU.

---

## 35. Hex-Band Concentric Color — 8-Level Radial Palette with Cube Distance (Hex Grid)

**Where:** `geometry/hex_grid.c`

**How it works:**
All color pairs for all themes are registered at startup in a 2D table `g_pal[N_THEMES][N_BAND_COLORS]`. The pair index for any hex cell is computed as `PAIR(theme, band) = theme * N_BAND_COLORS + band + 1`. This means switching themes requires no `init_pair` calls — only the draw call changes which pair index it uses:

```c
#define PAIR(t, b)  ((t) * N_BAND_COLORS + (b) + 1)

/* All 48 pairs registered once at startup */
for (int t = 0; t < N_THEMES; t++)
    for (int b = 0; b < N_BAND_COLORS; b++)
        init_pair(PAIR(t, b), g_pal[t][b], -1);

/* At draw time: select band from cube distance, theme from UI state */
int dist = hex_dist(hx, hy, cx_hex, cy_hex);
int band = dist % N_BAND_COLORS;
attron(COLOR_PAIR(PAIR(theme, band)));
```

The 8-band fire palette (`{231, 226, 220, 214, 208, 202, 196, 160}`) runs from white at the center to dark red at the outermost band — the inner rings glow brightest, fading outward. The modulo wraps distance: hex cells at distance 8 restart the same color as distance 0, creating repeating concentric rings at large grids.

**Effect:** Theme switching is instantaneous (no init_pair calls needed at runtime). The concentric ring pattern is an automatic consequence of the cube-distance metric — no explicit ring list is maintained.

---

## 36. HSV-to-RGB Fragment Shader — Iteration Depth as Hue (Mandelbulb Raster)

**Where:** `raster/mandelbulb_raster.c`

**How it works:**
The Mandelbulb surface has a "smooth" iteration count stored per vertex at mesh time. In the fragment shader, this smooth value drives a full HSV-to-RGB conversion: hue cycles through the spectrum N times across the smooth range, while value (brightness) comes from Blinn-Phong lighting. The two signals are orthogonal — depth determines hue, lighting determines brightness:

```c
static Vec3 hsv_to_rgb(float h, float s, float v)
{
    h *= 6.0f;
    int   i = (int)h;
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    switch (i % 6) {
    case 0: return v3(v, t, p);
    case 1: return v3(q, v, p);
    /* ... */
    }
}

/* In the fragment shader */
float hue  = fmodf(smooth * hue_bands, 1.0f);   /* N cycles across surface */
float luma = 0.10f + 0.72f * diffuse + 0.28f * specular;
Vec3  c    = hsv_to_rgb(hue, 0.82f, luma);
/* gamma encode before mapping to xterm-256 */
c.x = powf(c.x, 1.f / 2.2f);
```

The RGB float output is then quantized to the nearest xterm-256 6×6×6 cube index: `16 + 36*r + 6*g + b` where r, g, b ∈ [0, 5]. This is a 12-pair HUE_N=12 setup registered at startup (one pair per 30° hue step), not a per-pixel `init_pair` call.

**Effect:** Concentric depth shells of the Mandelbulb appear in different hues, giving the surface structure a "geologic strata" appearance — the viewer can see which features belong to the same iteration-depth shell, even as lighting varies. Adjustable `hue_bands` from 1–5 controls how many full spectrum cycles span the surface.

---

## 37. Algorithm-State Color Encoding — 4-Role Palette for Sort Visualization

**Where:** `misc/sort_vis.c`

**How it works:**
Rather than a gradient or continuous scale, the sort visualizer uses exactly four color roles, each representing the current algorithmic state of a bar in the array. The color assignments are mutually exclusive and applied per-column each frame:

```c
enum { CP_NORM=1, CP_CMP, CP_SWP, CP_SORT, CP_HUD };

init_pair(CP_NORM, 250, -1);   /* grey  — unsorted, not active     */
init_pair(CP_CMP,   51, -1);   /* cyan  — currently being compared */
init_pair(CP_SWP,  196, -1);   /* red   — just swapped this tick   */
init_pair(CP_SORT,  46, -1);   /* green — permanently sorted       */

int cp;
if      (i == g_swp1 || i == g_swp2) cp = CP_SWP;
else if (i == g_cmp1 || i == g_cmp2) cp = CP_CMP;
else if (g_done)                      cp = CP_SORT;
else                                  cp = CP_NORM;

/* A_BOLD on swapped bars makes the swap visually louder than a compare */
attron(COLOR_PAIR(cp) | (cp == CP_SWP ? A_BOLD : 0));
```

Only two indices (`g_cmp1`, `g_cmp2`) and two swap indices (`g_swp1`, `g_swp2`) are tracked. They are reset to -1 at the start of each step so only the current operation is highlighted — preventing stale highlights from persisting across ticks.

**Effect:** The eye immediately reads the algorithm's progress: grey mass (unsorted), cyan pair (comparing), red flash (swap), growing green region (sorted prefix). The algorithm's behaviour is legible even without reading the HUD counter.

---

## 38. Foreground + Background Both Set — Solid Fill for Maze Walls

**Where:** `misc/maze.c`

**How it works:**
In most ncurses animations, only the foreground color is set and the background is left transparent (`-1`). The maze visualizer intentionally sets both foreground and background for wall cells, creating solid filled blocks that visually read as thick walls rather than outlined lines. The solution path similarly uses a colored background to appear as a channel carved through space:

```c
init_pair(CP_WALL,      232, 232);   /* black on black — impenetrable wall */
init_pair(CP_CELL,      255, 235);   /* white fg, dark grey bg — open cell */
init_pair(CP_PATH,       51,  17);   /* cyan fg, dark blue bg — BFS path   */
init_pair(CP_DONE_PATH, 231,  22);   /* white fg, green bg — final path    */
```

The wall rendering draws space characters `' '` with the black-on-black pair, producing a block that visually matches the terminal background but is explicitly colored — important for terminals that don't inherit a white background.

**When to use:** Any visualization where structural regions (walls, barriers, backgrounds) should read as solid fills rather than drawn outlines. Setting both fg and bg to the same color is the minimal "block fill" technique.

---

## 39. Role-Based Pair Semantics — Robot/Mechanical Palette

**Where:** `animation/hexpod_tripod.c`

**How it works:**
Each color pair is assigned a fixed semantic role rather than a free color slot. This means the Theme struct just stores one color per role, and swapping themes changes all eight pairs simultaneously with a single `theme_apply()` call — the rendering code never changes.

The seven roles used in the hexapod:

| Pair | Role | Visual purpose |
|------|------|----------------|
| 1 | body frame | Rectangle edges, cross-braces, hip markers |
| 2 | femur | Hip → knee segment lines |
| 3 | tibia | Knee → foot segment lines |
| 4 | planted foot | `*` marker — bright accent, high contrast |
| 5 | stepping foot | `o` marker — dim, in-flight / transient |
| 6 | knee joint | `o` marker — bold, structural mid-joint |
| 7 | reserved | Unused; available for floor grid or debug |
| 8 | HUD | Status bar text — always high contrast |

```c
typedef struct {
    const char *name;
    int col[N_PAIRS];   /* foreground color index per role (pairs 1–7) */
    int hud;            /* pair 8 foreground */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name     body  femur  tibia  plant  step   knee  rsvd  hud  */
    {"Steel",  {245,  67,   75,   46,  214,  231,  238}, 226},
    {"Cobalt", {237,  27,   39,   46,  208,  231,  236}, 226},
    {"Copper", {242, 130,  172,   46,  214,  231,  238}, 220},
    {"Toxin",  {234,  34,   40,   46,   82,  231,  237},  46},
    {"Ember",  {239, 130,  136,   46,  208,  231,  238}, 208},
    {"Ghost",  {240, 251,  254,   46,  252,  255,  238}, 253},
    {"Neon",   {235,  93,  201,   46,  226,  255,  237}, 197},
    {"Ocean",  {234,  27,   51,   46,   51,  231,  237},  51},
};
```

The planted-foot pair (4) is always 256-color green (46) across every theme — it must be immediately visible regardless of which theme is active, since spotting which feet are down is the primary visual feedback for the gait.

**Design principle:** Assign roles before choosing colors. The question "what does this color communicate?" (planted/stepping/structural) is answered once in the role assignment; the per-theme colors then just tune aesthetics without breaking communication.

**When to use:** Any simulation with multiple visually distinct element classes that need to remain perceptually separable across all themes. Separating "what this color means" (role) from "what this color is" (palette) makes theme switching trivially safe.

**Effect:** The maze reads immediately as a physical structure — dark solid walls, light open passages, and a glowing blue-green solution path, without any box-drawing characters or Unicode blocks.

---

## 22. Curvature-Based Density Shading (beam_bending.c)

**Where:** `physics/beam_bending.c`

**How it works:** The beam deflection w(x) and bending moment M(x) are computed analytically. At each node, a character from the Paul Bourke density ramp `".,-~=+*#@"` is selected proportional to `|w[i]| / max_deflection`. Nodes with high curvature (large |w|, near load application points) get dense chars like `#` or `@`; nodes near supports get sparse chars like `.` or `,`. This communicates structural behavior — where the beam deforms most — through character density alone, without any secondary plot.

The bending moment panel uses separate color semantics: cyan for positive moment (sagging, tension at bottom) and magenta for negative moment (hogging, tension at top). This is a role-based assignment: the sign of M carries physical meaning that a neutral palette would obscure.

```c
/* beam density ramp — char encodes curvature magnitude */
static const char k_beam_ramp[] = ".,-~=+*#@";
#define BEAM_RAMP_N 9

/* in render_beam(): */
float frac = fabsf(b->w[i]) / (b->max_deflection + 1e-6f);
int   ri   = (int)(frac * (BEAM_RAMP_N - 1));
char  ch   = k_beam_ramp[ri];

/* moment panel color */
int cp = (b->M[i] >= 0.0f) ? CP_MOM_POS : CP_MOM_NEG;  /* cyan / magenta */
```

**Effect:** The beam reads as a physical object — denser characters where it flexes most, the moment diagram beside it showing sign changes. All information is carried in character choice and color, with no separate graph window.

---

## 23. Role-Named Velocity Arrow Colors (diff_drive_robot.c)

**Where:** `robots/diff_drive_robot.c`

**How it works:** Each visual element gets a named color pair constant, not a numbered pair. This separates the question "what does this color mean?" from "what xterm-256 index is it?":

```c
/* §3 color — role names, not numbers */
#define CP_HEAD   1   /* heading arrow — cyan      */
#define CP_VEL_P  2   /* positive velocity — green */
#define CP_VEL_N  3   /* negative velocity — red   */
#define CP_WHL_L  4   /* left wheel label — yellow */
#define CP_WHL_R  5   /* right wheel label — white */
#define CP_TR_NEW 6   /* fresh trail dots — bright */
#define CP_TR_OLD 7   /* old trail dots — dim      */
#define CP_HUD    8   /* HUD text                  */
```

Green velocity arrows mean "moving forward" and red means "reversing" — the color matches the intuition. The heading arrow is cyan to stand out from both velocity colors. Trail dots age from bright to dim using `A_BOLD` → `A_DIM` attribute gating on top of the same color pair, giving three brightness levels without extra pairs.

**Effect:** At a glance the viewer can read robot state: cyan arrow is "I'm going this way", green/red wheel arrows are "each wheel is pushing/braking", aged trail shows the robot's history.
