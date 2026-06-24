/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lenia.c — Lenia, a smooth version of Conway's Game of Life.
 *
 * Every cell holds a value between 0 and 1 instead of just on/off, and the
 * rules are smooth too. The payoff: lifelike blobs ("creatures") form on
 * their own and glide around the screen. Each step, a cell looks at how full
 * its surrounding ring of neighbours is, then grows or shrinks toward 1 or 0.
 *
 * References (the code can't tell you these):
 *   Chan, "Lenia: Biology of Artificial Life", Complex Systems 28(3), 2019.
 *     arXiv:1812.05433 — the paper that defines the ring kernel + growth rule.
 *   Rafler, "SmoothLife", 2011. arXiv:1111.1567 — the continuous-Life
 *     precursor this update rule builds on.
 *   Bourke, "Character Representation of Grey Scale Images", 1997.
 *     paulbourke.net/dataformats/asciiart — the . : + * # brightness ramp.
 *
 * §1 config+types  §2 performance  §3 logic  §4 simulation  §5 render  §6 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config + types — the one place all data is declared ── */

#define GRID_W_MAX  200
#define GRID_H_MAX   60
#define HUD_TOP       1   /* data row at top      */
#define HUD_BOT       1   /* action row at bottom */
#define KERNEL_R_MAX 20   /* max kernel radius in cells */
#define RENDER_NS   (1000000000LL / 30)
#define ACTIVITY_MIN 0.003f   /* if the average cell gets this faint, the field is dead → reseed */

#define SPEED_MAX         4       /* most simulation steps we'll run per drawn frame */

/* the neighbour ring's shape */
#define RING_THICKNESS    0.2f    /* how thick the ring is, as a fraction of its radius */
#define KERNEL_WEIGHT_MIN 0.001f  /* skip ring cells lighter than this so the loop stays short */

/* seeding & drawing */
#define SEED_DISC_RATIO   0.8f    /* the center seed fills a disc this fraction of R wide */
#define CELL_DRAW_MIN     0.05f   /* cells dimmer than this are drawn as blank */

/* Species — the four numbers that make one kind of Lenia creature.
 *
 * In Lenia a "creature" IS just these four numbers; Orbium and Aquarium run
 * the exact same code and only differ here. So switching presets is nothing
 * more than copying a Species. Every step, each cell does:
 *     new = clip( old + dt * (2*growth(neighbour_fullness) - 1) ) */
typedef struct {
    /* the neighbour-fullness the creature is happiest at: when a cell's ring
     * is this full, it grows hardest. Higher = it wants denser company.
     * Usually 0.10–0.30. */
    float mu;
    /* how picky it is about that target. Small = fragile, dies if it's even a
     * little off; large = forgiving and blobby. Usually 0.01–0.04. */
    float sigma;
    /* how far out each cell looks, in cells — the creature's size. Also sets
     * the per-cell work; capped at KERNEL_R_MAX when the ring is built. */
    float R;
    /* step size: how much of the grow/shrink we apply each step. Smaller =
     * smoother and slower; too big and the values blow up. Usually 0.10–0.15. */
    float dt;
} Species;

/* Preset — one named creature in the catalogue, picked by keys 1-3. Same
 * numbers as Species plus a name. The name is only here so the HUD can label
 * what's running; the simulation itself never needs it. All three are classic
 * Lenia creatures from the Chan 2019 paper. */
typedef struct {
    const char *name;    /* label for the HUD, e.g. "Orbium" */
    Species     species; /* the rule this entry loads */
} Preset;

static const Preset PRESETS[] = {
    { "Orbium",   { 0.150f, 0.015f, 13.f, 0.10f } },
    { "Aquarium", { 0.260f, 0.036f, 10.f, 0.10f } },
    { "Scutium",  { 0.170f, 0.015f,  8.f, 0.15f } },
};

/* Colour-pair IDs for ncurses. CP_U0..CP_U4 are a 5-step ramp from dim to
 * bright for the cell value (set in color_init, picked in cell_glyph); CP_HUD
 * and CP_HINT colour the two text rows. Starts at 1 because ncurses keeps
 * pair 0 for the terminal's own default colours. */
enum { CP_U0=1, CP_U1, CP_U2, CP_U3, CP_U4, CP_HUD, CP_HINT };

/* Field — the world. A grid where each cell holds a value from 0 (empty) to 1
 * (full). Unlike plain Game of Life's on/off cells, these in-between values
 * are what let smooth gliding creatures exist.
 *
 * Two grids (cur + next): a step has to read all neighbours from the same
 * frozen snapshot, so we write results into `next` and copy it over `cur`
 * only when the whole pass is done. Updating in place would let just-changed
 * cells poison neighbours we haven't computed yet.
 *
 * Edges wrap around (see field_potential): a creature gliding off one side
 * comes back on the other, so there are no walls to bump into. */
typedef struct {
    /* the real world: every cell's value, 0 to 1. Everyone reads this. */
    float cur [GRID_H_MAX][GRID_W_MAX];
    /* scratch space where one step builds the next frame before copying it
     * back into cur. Nothing but field_step ever touches it. */
    float next[GRID_H_MAX][GRID_W_MAX];
    /* the part we actually use, sized to fit the window. The full arrays stay
     * MAX-sized so we never have to reallocate; a small window just uses less. */
    int   h, w;
} Field;

/* Kernel — the "ring" of neighbours each cell senses. Most of the weight sits
 * on a thin circular shell at radius R; the middle and the far corners count
 * for ~nothing. Summing a cell's neighbours through this ring tells it how
 * full its surroundings are — the sensing half of the rule. The ring shape is
 * Lenia's signature (Chan 2019), a smoother take on SmoothLife's ring.
 *
 * Stored sparsely — three matching arrays plus a count, not a full square
 * grid. Since only the shell carries weight, we keep just the cells worth
 * adding and loop over those `n`, which keeps the hot path short. */
typedef struct {
    /* weight of ring cell i, scaled so all weights add up to 1. That makes
     * the sum a weighted average, so the fullness stays in the same 0..1
     * range as the cells themselves. */
    float w [(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
    /* where ring cell i sits relative to the cell we're updating:
     * dr rows away, dc columns away (either can be negative). */
    int   dr[(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
    int   dc[(2*KERNEL_R_MAX+1)*(2*KERNEL_R_MAX+1)];
    /* how many ring cells we actually kept; depends on R, rebuilt each time. */
    int   n;
} Kernel;

/* Scene — everything the simulation needs, bundled into one struct that the
 * §6 driver functions pass around. Grouped so it reads like a contents page:
 * WHAT is being simulated, HOW it's driven, and WHERE on screen. Bundling it
 * here is just convenience — the worker functions still take only the narrow
 * piece they need (a Field*, a Species*, …), so the layers stay independent. */
typedef struct {
    /* WHAT — the world plus the ring that evolves it. */
    Field   field;
    Kernel  kernel;
    /* HOW — the live rule and the bits of UI state around it. */
    Species species;     /* the running rule — a copy of one preset */
    int     preset;      /* which preset is loaded, for the HUD label */
    int     speed;       /* simulation steps per drawn frame (1–4) */
    /* WHERE — screen size and pause state; too small to need their own types. */
    int     rows, cols;  /* terminal size right now, in characters */
    bool    paused;      /* true = stop simulating but keep drawing */
} Scene;

static Scene g_scene = { .speed = 1 };

/* How the signal handler talks to the main loop. Not part of the Scene. */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ── §2  performance — the clock and the frame-rate sleep ── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3  logic — pure helpers: they only read, never change anything ── */

/* Pull a cell value back inside 0..1 if it strayed out. */
static float clamp01(float v)
{
    return v < 0.f ? 0.f : v > 1.f ? 1.f : v;
}

/* Is this neighbour offset within `radius` cells of the center? */
static bool within_radius(int dr, int dc, float radius)
{
    return sqrtf((float)(dr*dr + dc*dc)) <= radius;
}

static int imin(int a, int b) { return a < b ? a : b; }

/* How strongly a neighbour at distance `dist` counts toward the ring: full at
 * the ring's radius R, fading out on either side. */
static float ring_weight(float dist, float R)
{
    float x = (dist - R) / (RING_THICKNESS * R + 0.001f);  /* tiny add so R=0 won't divide by zero */
    return expf(-x*x * 0.5f);
}

/* The grow-or-shrink response for a cell whose ring is U full: peaks when U
 * matches the creature's preferred fullness, drops off as it strays. */
static float growth(const Species *sp, float U)
{
    float x = (U - sp->mu) / sp->sigma;
    return expf(-x*x * 0.5f);
}

/* How full cell (r,c)'s neighbour ring is: add up the ring neighbours, each
 * scaled by its weight. Edges wrap around, so corners have full rings too. */
static float field_potential(const Field *f, const Kernel *k, int r, int c)
{
    float U = 0.f;
    for (int i = 0; i < k->n; i++) {
        int nr = (r + k->dr[i] + f->h) % f->h;   /* wrap past the top/bottom edge */
        int nc = (c + k->dc[i] + f->w) % f->w;   /* wrap past the left/right edge */
        U += k->w[i] * f->cur[nr][nc];
    }
    return U;
}

/* Average cell value — close to zero means everything has faded out. */
static float field_activity(const Field *f)
{
    float sum = 0.f;
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            sum += f->cur[r][c];
    return sum / (float)(f->h * f->w);
}

/* ── §4  simulation — the parts that change the world ── */

static void kernel_add_entry(Kernel *k, int dr, int dc, float w)
{
    int e = k->n++;
    k->dr[e] = dr; k->dc[e] = dc; k->w[e] = w;
}

/* Rescale the ring weights so they add up to 1, turning the sum into an
 * average and keeping fullness in the same 0..1 range as the cells. */
static void kernel_normalize(Kernel *k)
{
    float total = 0.f;
    for (int e = 0; e < k->n; e++) total += k->w[e];
    if (total > 0.f)
        for (int e = 0; e < k->n; e++) k->w[e] /= total;
}

/* Build the neighbour ring for radius R: walk the square box around a cell,
 * keep the cells that land on the ring, drop the rest, then normalise. */
static void kernel_build(Kernel *k, float R)
{
    int iR = imin((int)ceilf(R), KERNEL_R_MAX);   /* how far out to scan */
    k->n = 0;
    for (int dr = -iR; dr <= iR; dr++)
        for (int dc = -iR; dc <= iR; dc++) {
            float dist = sqrtf((float)(dr*dr + dc*dc));
            float w    = ring_weight(dist, R);
            if (w >= KERNEL_WEIGHT_MIN)
                kernel_add_entry(k, dr, dc, w);
        }
    kernel_normalize(k);
}

/* Fill the whole grid with random static. Used by 'r' and the auto-reseed,
 * never by the regular step. */
static void field_randomize(Field *f)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++)
            f->cur[r][c] = (float)rand()/RAND_MAX;
}

static void field_clear(Field *f)
{
    memset(f->cur, 0, sizeof f->cur);
}

/* Drop a random blob in the middle: a disc about 0.8*R wide filled with random
 * values, everything else blank. A fresh creature usually grows out of this. */
static void field_seed_center(Field *f, const Species *sp)
{
    field_clear(f);
    int cr = f->h/2, cc = f->w/2;     /* middle cell */
    int R  = (int)sp->R;              /* how far out to scan */
    for (int dr = -R; dr <= R; dr++)
        for (int dc = -R; dc <= R; dc++) {
            int r = cr + dr, c = cc + dc;
            if (r < 0 || r >= f->h || c < 0 || c >= f->w) continue;  /* skip off-grid */
            if (within_radius(dr, dc, R * SEED_DISC_RATIO))
                f->cur[r][c] = (float)rand()/RAND_MAX;
        }
}

/* One step of the whole world. Each cell checks how full its ring is, then
 * grows (toward 1) or shrinks (toward 0) a little. Results go into `next` so
 * every cell sees the same starting snapshot, then `next` is copied over. */
static void field_step(Field *f, const Kernel *k, const Species *sp)
{
    for (int r = 0; r < f->h; r++)
        for (int c = 0; c < f->w; c++) {
            float U  = field_potential(f, k, r, c);           /* how full the ring is */
            float du = sp->dt * (2.f * growth(sp, U) - 1.f);  /* grow or shrink, and by how much */
            f->next[r][c] = clamp01(f->cur[r][c] + du);
        }
    memcpy(f->cur, f->next, sizeof f->cur);
}

/* ── §5  render — turn the world into characters on screen ── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_U0, 25, -1);   /* dark navy  — faintest cells */
        init_pair(CP_U1,  27, -1);   /* blue */
        init_pair(CP_U2,  51, -1);   /* cyan */
        init_pair(CP_U3, 195, -1);   /* pale cyan */
        init_pair(CP_U4, 231, -1);   /* white      — fullest cells */
        init_pair(CP_HUD, 226, -1);  /* bright yellow — top info row */
        init_pair(CP_HINT, 51, -1);  /* bright cyan   — bottom key row */
    } else {
        init_pair(CP_U0, COLOR_BLUE,   -1);
        init_pair(CP_U1, COLOR_BLUE,   -1);
        init_pair(CP_U2, COLOR_CYAN,   -1);
        init_pair(CP_U3, COLOR_CYAN,   -1);
        init_pair(CP_U4, COLOR_WHITE,  -1);
        init_pair(CP_HUD, COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,  -1);
    }
}

/* Pick the character and colour for a cell by how full it is: fuller cells get
 * a denser, brighter glyph (the . : + * # ramp, Bourke). Returns false for
 * near-empty cells so they stay blank. */
static bool cell_glyph(float u, chtype *glyph, int *pair)
{
    if      (u < CELL_DRAW_MIN) return false;
    else if (u < 0.20f) { *pair = CP_U0; *glyph = '.'; }
    else if (u < 0.40f) { *pair = CP_U1; *glyph = ':'; }
    else if (u < 0.60f) { *pair = CP_U2; *glyph = '+'; }
    else if (u < 0.80f) { *pair = CP_U3; *glyph = '*'; }
    else                { *pair = CP_U4; *glyph = '#'; }
    return true;
}

/* Draw the grid in the area between the top and bottom info rows. */
static void draw_field(const Field *f, int rows, int cols)
{
    for (int r = 0; r < f->h && r + HUD_TOP < rows - HUD_BOT; r++)
        for (int c = 0; c < f->w && c < cols; c++) {
            chtype glyph; int pair;
            if (!cell_glyph(f->cur[r][c], &glyph, &pair)) continue;
            attron(COLOR_PAIR(pair));
            mvaddch(r + HUD_TOP, c, glyph);
            attroff(COLOR_PAIR(pair));
        }
}

/* Draw one info row at `row`, cut off at the window width so it can't spill
 * onto the next line. */
static void draw_hud_row(int row, int pair, int cols, const char *text)
{
    char buf[160];
    snprintf(buf, sizeof buf, "%s", text);
    if ((int)strlen(buf) > cols) buf[cols] = '\0';
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Top row shows the live settings; bottom row lists the keys. We spell out
 * "mu"/"sig" instead of μ/σ so it prints on any terminal. */
static void draw_hud(const Scene *sc)
{
    const Species *sp = &sc->species;
    char data[160];
    snprintf(data, sizeof data,
        " Lenia [%d] %-8s mu=%.3f sig=%.3f R=%.0f dt=%.2f  speed:%dx  %s ",
        sc->preset+1, PRESETS[sc->preset].name,
        sp->mu, sp->sigma, sp->R, sp->dt,
        sc->speed, sc->paused ? "PAUSED " : "running");
    draw_hud_row(0, CP_HUD, sc->cols, data);
    draw_hud_row(sc->rows - 1, CP_HINT, sc->cols,
        " q:quit  1/2/3:preset  p:pause  r:random  +/-:speed  spc:seed ");
}

/* Draw everything: the grid, then the two info rows. */
static void scene_draw(const Scene *sc)
{
    draw_field(&sc->field, sc->rows, sc->cols);
    draw_hud(sc);
}

/* ── §6  app — input, the main loop, and the glue that drives it all ── */

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void install_signals(void)
{
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);
}

/* Set up the terminal for animation: read keys without waiting, no echo, no
 * blinking cursor. */
static void terminal_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
}

/* Resize the working grid to match the window, but never past the fixed
 * arrays' size. */
static void scene_fit_terminal(Scene *sc)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    sc->rows = rows; sc->cols = cols;
    sc->field.h = imin(rows - HUD_TOP - HUD_BOT, GRID_H_MAX);
    sc->field.w = imin(cols, GRID_W_MAX);
}

/* Switch to preset `p`: load its numbers, rebuild the ring, start fresh. */
static void scene_load_preset(Scene *sc, int p)
{
    sc->preset  = p;
    sc->species = PRESETS[p].species;
    kernel_build(&sc->kernel, sc->species.R);
    field_randomize(&sc->field);   /* presets start as random static, same as 'r' */
}

/* Advance the simulation one frame's worth — the only place the world moves.
 * Runs `speed` steps, and if everything has died out, reseeds with static. */
static void scene_tick(Scene *sc)
{
    if (sc->paused) return;
    for (int s = 0; s < sc->speed; s++)
        field_step(&sc->field, &sc->kernel, &sc->species);
    if (field_activity(&sc->field) < ACTIVITY_MIN)
        field_randomize(&sc->field);
}

/* Window was resized: rebuild ncurses, refit the grid, start fresh. */
static void handle_resize(Scene *sc)
{
    g_resize = 0;
    endwin(); refresh();
    scene_fit_terminal(sc);
    scene_load_preset(sc, sc->preset);
}

/* Act on one key press. */
static void handle_key(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1; break;
    case 'p': case 'P': sc->paused = !sc->paused; break;
    case 'r': case 'R': field_randomize(&sc->field); break;
    case ' ': field_seed_center(&sc->field, &sc->species); break;
    case '1': scene_load_preset(sc, 0); break;
    case '2': scene_load_preset(sc, 1); break;
    case '3': scene_load_preset(sc, 2); break;
    case '+': case '=': sc->speed++; if (sc->speed > SPEED_MAX) sc->speed = SPEED_MAX; break;
    case '-': sc->speed--; if (sc->speed < 1) sc->speed = 1; break;
    default: break;
    }
}

/* Wipe, draw, and push the frame to the screen in one update. */
static void frame_render(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    install_signals();
    terminal_init();

    scene_fit_terminal(&g_scene);
    scene_load_preset(&g_scene, 0);

    while (!g_quit) {
        if (g_resize) handle_resize(&g_scene);
        handle_key(&g_scene, getch());

        long long frame_start = clock_ns();
        scene_tick(&g_scene);
        frame_render(&g_scene);

        long long elapsed = clock_ns() - frame_start;
        clock_sleep_ns(RENDER_NS - elapsed);   /* nap the leftover time to hold a steady frame rate */
    }
    return 0;
}
