# Architecture — The Big Ideas

Every program here is a tiny world that redraws itself dozens of times a second
inside your terminal. Open the hood on enough of them and you find the same
handful of tricks underneath. This document is the friendly tour of those
tricks — *what* each one is, *why* it exists, and the *core idea* that makes it
tick.

It deliberately shows **almost no code**. Every `.c` file is heavily commented
and walks you through its own math; the shared skeleton is explained in
`ncurses_basics/framework.c` and in **Framework.md**; and **DEMOS.md** lists
every demo with a one-line note. Read this first to get the ideas in plain
language — then open any file and it will feel familiar.

> **Where to find what**
> - *The framework / loop, line by line* → `framework.c` + **Framework.md**
> - *A specific demo's full detail* → that file's own header (CONCEPTS + MENTAL MODEL)
> - *The catalogue of every program* → **DEMOS.md**
> - *The recurring ideas, explained simply* → you're reading it

---

## The shared skeleton

### Keeping time — the fixed-timestep loop
**What.** A way to make things move at the *same* speed on a fast computer and a
slow one.
**Why.** If you just nudge things "a bit each frame," a fast machine runs frames
quicker and everything zooms; a slow one crawls — and the physics can blow up.
We want the simulation's heartbeat to be steady no matter how fast the screen
can draw.
**Core concepts.** Keep a little *bucket* of leftover time. Each real frame, pour
in the time that passed. While the bucket holds at least one "tick," advance the
world by exactly one fixed step and scoop that much out. Drawing happens as often
as it can; *thinking* always happens in equal steps. When the screen draws
between two ticks, blend the last two positions so motion stays smooth — that
blend amount is `alpha`.
→ `ncurses_basics/framework.c`

### Round things in a grid of letters — coordinate spaces & the aspect fix
**What.** A way to keep circles round and motion smooth when your "pixels" are
fat letters.
**Why.** A terminal cell is about twice as tall as it is wide. Treat rows and
columns the same and a "circle" comes out as a squashed egg. And smooth movement
needs fractional positions the whole-number grid can't store directly.
**Core concepts.** Do the math in a fine *pixel space* (each cell is
`CELL_W × CELL_H` sub-pixels) and convert to a row/column only at the very moment
of drawing. Multiply vertical distances by the cell's tallness (~2) so shapes
come out round. There is exactly *one* place that converts pixel → cell;
nowhere else.
→ `ncurses_basics/framework.c` (§4 coords), `physics/bounce_ball.c`

### Drawing with letters — the brightness ramp
**What.** Turning a number — how bright, hot, or dense a spot is — into a
character.
**Why.** A terminal can't shade a pixel; it can only print letters. But letters
cover different amounts of ink: a space is empty, `.` is faint, `@` is almost
solid. Lined up from light to heavy, they *become* a grayscale.
**Core concepts.** Pick a ramp like `` .:-=+*#@`` (light → heavy). Squash your
value to the range 0..1, multiply by the ramp length, and pick that character.
Colour can ride on top — a heat ramp from dark red to white-hot. This one move,
*number → ramp slot → glyph*, is how every demo shades the screen.
→ `raster/*.c` (the spinning solids), the particle and fluid demos

---

## Simulating many things

### Lots of little things — the particle pool
**What.** One fixed box of (say) 1024 reusable specks — sparks, raindrops, bomb
fragments.
**Why.** Making and destroying thousands of objects *while* the animation runs is
slow and fiddly. So we build them all once, up front, and just flip each one on
or off.
**Core concepts.** Every particle carries an `active` flag. To spawn one, grab a
dead slot and switch it on. Each tick, every live particle moves, ages, and
cools; when it's too old or drifts off-screen it switches *itself* off — free for
the next spark. The box never grows; it recycles. Fire, fireworks, rain, snow,
the volcano's lava bombs — all the same box of reusable dots.
→ `particle_systems/fire.c`, `artistic/volcano.c`

### Worlds on a grid — cellular automata & the double buffer
**What.** A grid where each cell's next state depends on its neighbours — fire
spreading, Conway's Life, ripples crossing a pond.
**Why.** If you update cells in place, a cell you *just* changed pollutes its
neighbour's calculation — the result then depends on which order you visited
cells, which is a sneaky bug.
**Core concepts.** Keep **two** grids: read the old one, write the new one, then
swap them. Every cell sees the same frozen "previous" world, so the rule is fair
and order doesn't matter. That read-old / write-new / swap dance is the
"double-buffer wavefront."
→ `procedural/generational/life.c`, `particle_systems/fire.c`

### Stuff that flows — continuous fields & PDEs
**What.** Smoke, water, heat, spreading chemical patterns — a quantity smeared
over space that changes over time.
**Why.** These obey physics equations (Navier-Stokes, diffusion) describing how
each point pushes its neighbours. We can't solve them perfectly, so we
approximate them on the grid.
**Core concepts.** Store the quantity in every cell. Each tick, update a cell
from its neighbours with a small rule that imitates the equation (a *stencil* —
e.g. "become a bit more like the average of my neighbours" is diffusion). Keep
the timestep tiny or it explodes — the stability limit is the CFL condition, and
one demo exists just to let you cross it and watch the blow-up.
→ `fluid/navier_stokes.c`, `procedural/fields/reaction_diffusion_gray_scott.c`

### Believable randomness — noise (value / Perlin / fBm)
**What.** Random-looking but *smooth* wiggle — for clouds, terrain, plasma, the
volcano's plume.
**Why.** Pure randomness (white noise) is harsh TV static. Nature is random but
smooth — rolling hills, soft clouds. We want randomness with no jarring jumps.
**Core concepts.** Scatter random values on a coarse grid of points and blend
smoothly between them → smooth noise. Stack several layers, each one finer and
fainter (the *octaves* of fBm), and you get natural-looking fractal detail. The
same seed always regenerates the same pattern.
→ `procedural/fields/value_noise_showcase.c`, `procedural/fields/simplex_noise_clouds.c`

### Building worlds from a seed — procedural generation
**What.** Mazes, dungeons, terrain, galaxies — all grown from a single random
seed.
**Why.** Hand-designing every map is endless; a recipe plus a seed makes infinite
ones, and the same seed reproduces the same world exactly.
**Core concepts.** Two flavours. (a) *Build* something step-by-step and then stop
— carve a maze, drop rooms, collapse tiles — usually driven by a little state
machine (`BUILDING → HOLD → reset`). (b) *Evaluate* a pure function of
position-plus-seed everywhere at once — a noise landscape, a star field — where
there's nothing to build, you just ask "what's here?"
→ `procedural/generational/maze_backtracker.c`, `procedural/worldgen/*.c`

---

## Drawing in 3-D

### 3-D from a distance function — SDF sphere-marching
**What.** Drawing 3-D shapes — spheres, blobs, fractals — with no triangles at
all.
**Why.** If you describe a shape by "how far am I from its surface?" (a Signed
Distance Function), you can bend, blend, dent, and endlessly repeat shapes with
simple arithmetic — things that are painful with triangle meshes.
**Core concepts.** From the eye, shoot one ray per pixel. Ask the SDF "how far to
the nearest surface?" and step forward exactly that far — you're guaranteed not
to pass through anything. Repeat until you're basically touching it (a hit) or
you've travelled too far (a miss). For lighting, the surface's facing direction
is simply *which way the distance grows fastest*.
→ `raymarcher/raymarcher.c`, `raymarcher/sdf_gallery.c`

### 3-D from triangles — the software rasteriser
**What.** The classic graphics-card pipeline — spinning cube, shaded models, the
famous donut — done by hand on the character grid.
**Why.** To really feel how a 3-D scene becomes a 2-D picture (projection,
shading, hidden surfaces), you build the assembly line yourself instead of
calling a GPU.
**Core concepts.** For each triangle: move its corners with the camera matrix,
project 3-D down to 2-D, discard the ones facing away, then fill in the pixels
inside it. A *depth buffer* remembers the nearest surface so far, so closer
triangles correctly hide farther ones. Each filled pixel gets a brightness from
a light — and then the brightness → glyph ramp paints it.
→ `raster/*.c`

### 3-D by solving for the hit — analytic raytracing
**What.** Like sphere-marching, but the exact ray-meets-shape point is found with
algebra, and light is allowed to bounce.
**Why.** For simple shapes you don't need to creep forward in steps — you can
solve a little equation (a quadratic, for a sphere) for the exact hit. And once
rays can bounce, you get real reflections, shadows, and soft lighting.
**Core concepts.** Shoot a ray, solve where it strikes each shape, keep the
closest. For a *path tracer*, let the ray ricochet randomly off surfaces many
times and average the results — speckly at first, then photo-smooth as the
samples pile up (this averaging of random bounces is "Monte Carlo").
→ `raytracing/sphere_raytrace.c`, `raytracing/path_tracer.c`

---

## Motion & structure

### Joints that move — FK / IK kinematic chains
**What.** Arms, legs, snakes, tentacles built from connected segments.
**Why.** There are two ways to pose a chain. *Forward*: set every joint's angle
and see where the tip ends up. *Inverse*: say where the tip *should* be and work
out the angles — harder, but it's what you need to make a limb "reach for that
point."
**Core concepts.** Forward kinematics walks the chain from the root: each segment
turns relative to its parent, and positions stack up. Inverse kinematics nudges
the joints over and over to pull the tip toward a target — e.g. FABRIK: drag the
chain out to the goal, then back to the root, and repeat until it settles.
→ `animation/fk_helloworld.c`, `animation/fk_ik_helloworld.c`

### Thirty demos from one function — data-driven preset tables
**What.** "30 mandalas," "10 DNA forms," "21 Fourier shapes" — all produced by a
*single* draw function.
**Why.** Writing thirty separate drawers means thirty times the bugs. Writing one
flexible drawer plus thirty little rows of numbers means there's only one drawer
to get right.
**Core concepts.** Make a table where each row is just a set of parameters (radii,
counts, colours, flags). The one draw function reads a row and renders it.
Switching presets only changes *which row* you read — no new code runs. Adding a
brand-new demo is adding a row.
→ `artistic/hindu_mandalas.c`, `artistic/dna.c`, `signal/fourier_shapes.c`

### Keeping big files readable — layer-separated architecture
**What.** A house style for arranging each program so a stranger can read it
top-to-bottom and understand it.
**Why.** When the math, the state, the update step, and the drawing all get
tangled together, every change risks breaking something far away. Separating them
by *job* keeps cause and effect side by side.
**Core concepts.** Split the file into labelled layers: *config* (all the knobs
in one place), *logic* (pure math with no side effects), *simulation* (the one
place that advances the world each tick), and *render* (reads the state and
draws, changing nothing). A function that only *reads* takes a look-don't-touch
(`const`) pointer; only a function that *changes* state gets a writable one. Name
things after the concept (a `Boid`, not an `Entity`). Done well, each function
reads like a short recipe.
→ `artistic/volcano.c`, `artistic/islamic_mandalas.c`

---

*That's the toolkit. Almost every program in this repository is some combination
of the ideas above, sitting on the shared fixed-timestep skeleton. For the loop
itself, read Framework.md; for any single demo, read its file header; for the
full list, see DEMOS.md.*
