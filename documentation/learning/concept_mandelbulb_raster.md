# Pass 1 — mandelbulb_raster.c: Mandelbulb via Software Rasterizer

## Core Idea

Instead of raymarching (one DE evaluation per march step, hundreds of steps per pixel), this file **tessellates** the Mandelbulb surface into a triangle mesh at startup, then renders it through the standard vertex/fragment shader pipeline from cube_raster.c.

The tradeoff is fundamental: **rasterization captures only the outer surface** (one layer of triangles, convex hull), while raymarching can penetrate into interior crevices and see every depth shell. The rasterizer is faster per frame but misses the fractal's interior structure.

---

## Sphere Projection Tessellation

The Mandelbulb has no analytic parametrisation. To get a mesh, we **project inward** from a UV sphere:

```
For each (lat θ, lon φ) on the UV sphere:
  Start at r = 1.5 (outside fractal)
  March inward along the radial direction
  First point where DE < MB_HIT_EPS → surface vertex
```

```c
for (int i = 0; i <= NLAT; i++) {
    float theta = i * M_PI / NLAT;
    for (int j = 0; j <= NLON; j++) {
        float phi = j * 2*M_PI / NLON;
        Vec3 dir = v3(sin(theta)*cos(phi), cos(theta), sin(theta)*sin(phi));
        /* march inward from r=1.5 */
        for (float r = 1.5f; r > 0.05f; r -= 0.015f) {
            Vec3 p = v3_scale(dir, r);
            float de = mb_de(p, power, MB_MARCH_ITERS, &smooth);
            if (de < MB_HIT_EPS) { record_vertex(p, smooth); break; }
        }
    }
}
```

Valid neighboring vertices are connected into quads → 2 triangles each. Invalid vertices (where the march didn't converge) produce no triangles.

### Why sphere projection misses interior

The outward normal of the outer fractal surface faces roughly away from the origin. Sphere projection follows radial rays, so it hits the outermost layer only. The inner pods, the cavities between tentacles, the re-entrant surfaces — none of these are captured. Comparing rasterizer output to raymarcher output makes this limitation immediately obvious.

---

## Three Fragment Shaders

### frag_phong_hue

Blinn-Phong lighting combined with HSV color from the smooth escape value:

```c
float hue = fmodf(in->custom[0] * u->hue_bands, 1.0f);
Vec3 surface_color = hsv_to_rgb(hue, 0.82f, luma);
```

`luma` is computed from the diffuse + specular lighting equation. The hue comes from the fractal's escape time at that vertex. Result: the fractal surface has both 3D shading depth and escape-time color simultaneously.

### frag_normals

Maps the world-space surface normal to hue:

```c
float azimuth = atan2f(N.z, N.x) / (2*PI) + 0.5f;
float value   = 0.45f + 0.55f * (N.y * 0.5f + 0.5f);
out->color = hsv_to_rgb(azimuth, 0.80f, value);
```

Azimuth angle (0→360°) maps to full hue cycle; Y component of normal controls brightness. Each unique surface orientation gets a distinct color — useful for debugging mesh quality.

### frag_depth_hue

Pure escape-time → hue, no lighting. This shader makes the rasterized Mandelbulb directly comparable to `mandelbulb_raymarcher.c`:

```c
float hue = fmodf(in->custom[0] * u->hue_bands, 1.0f);
out->color = hsv_to_rgb(hue, 0.90f, 0.88f);
```

Running both programs side-by-side with this shader mode shows exactly what the rasterizer captures (outer surface only) vs what the raymarcher captures (all depth shells).

---

## HSV → RGB

```c
static Vec3 hsv_to_rgb(float h, float s, float v) {
    h *= 6.0f;
    int   i = (int)h;
    float f = h - (float)i;
    float p = v*(1-s), q = v*(1-s*f), t = v*(1-s*(1-f));
    switch (i%6) {
    case 0: return v3(v,t,p);  case 1: return v3(q,v,p);
    case 2: return v3(p,v,t);  case 3: return v3(p,q,v);
    case 4: return v3(t,p,v);  case 5: return v3(v,p,q);
    }
}
```

`h ∈ [0,1)` maps once around the color wheel. `s=0.82` (saturation) and `v=luma` (value/brightness) give vivid but not washed-out colors.

---

## rgb_to_cell — Full-color Framebuffer

cube_raster.c uses `luma_to_cell` which maps luminance → one of 7 fixed color pairs. For mandelbulb_raster, we need the full 12-hue color pair set to represent any fragment color:

```c
static Cell color_to_cell(Vec3 color, int px, int py) {
    float luma = 0.2126f*color.x + 0.7152f*color.y + 0.0722f*color.z;
    /* Bayer dither for char selection */
    float d = luma + (k_bayer[py&3][px&3] - 0.5f) * 0.15f;
    char ch = k_bourke[(int)(d * (BOURKE_LEN-1))];
    /* Extract hue from RGB → nearest of 12 terminal color pairs */
    int cp = rgb_to_pair(color);
    return (Cell){ ch, cp, luma > 0.5f };
}
```

The `rgb_to_pair` function computes the hue angle of the fragment color and maps it to the nearest of the 12 pre-defined hue pairs (red, orange, amber, yellow, lime, green, teal, cyan, sky, blue, violet, magenta). Luminance still controls the character density (Bourke ramp), but the terminal color now faithfully represents the fragment's actual hue.

---

## Non-Obvious Decisions

### Power adjustment rebuilds the mesh

Pressing `p/P` changes the Mandelbulb power parameter and re-tessellates the entire mesh. The sphere projection tessellation runs in the main loop (before rendering starts for that frame). For NLAT=28, NLON=56, this is ~1600 sphere projection marches — takes ~50ms at low iteration counts, which causes a visible pause. This is intentional: the user sees the mesh change as power sweeps from 2 to 12.

### Why NLAT=28 × NLON=56 specifically?

NLON = 2 × NLAT keeps the aspect ratio of the UV grid cells roughly square before projection. At 28 latitude rings, there are enough rows for smooth silhouette curvature in the terminal. More rings increase tessellation time without visible improvement at ASCII resolution.

### Back-face culling toggle ('c')

The sphere projection mesh has a consistent winding convention. If the winding appears wrong for a particular power value (some surface regions face inward due to the projection), toggling culling off shows both sides. For power=8 (standard Mandelbulb) the winding is correct with culling on.
