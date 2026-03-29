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

## 13. xterm-256 Palette Index Reference

**How it works:**
The xterm-256 color space has three regions:
- **0–15:** Standard and high-intensity named colors
- **16–231:** 6×6×6 RGB cube — index = `16 + 36*r + 6*g + b` where r,g,b ∈ [0,5]
- **232–255:** 24-step grey ramp from near-black to near-white

Specific indices are chosen for maximum visibility on black backgrounds.

**Commonly used indices in this project:**
| Index | Color            | Used in               |
|-------|------------------|-----------------------|
| 196   | Bright red       | fire, fireworks       |
| 202   | Fire orange      | fire themes           |
| 214   | Amber            | fire, gold theme      |
| 226   | Bright yellow    | fire, gold theme      |
| 46    | Matrix green     | nova theme, matrix    |
| 51    | Bright cyan      | ice theme             |
| 33    | Dodger blue      | flocking              |
| 201   | Hot magenta      | plasma theme          |
| 232   | Near-black       | theme backgrounds     |
| 235–255 | Grey ramp steps | donut, rasters      |

**Effect:** Predictable, terminal-portable colors without relying on named color constants that vary by terminal theme.
