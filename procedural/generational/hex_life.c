/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hex_life.c — Conway's Game of Life, but on a hexagonal grid.
 *
 * Every cell has 6 neighbours instead of 8, and the rule is B2/S34: a dead
 * cell is born with exactly 2 live neighbours, a live cell survives on 3 or 4.
 * The hex stagger is faked by shifting odd rows right one column when drawn.
 *
 * The hex rule on a hex tiling comes from Bays, "A Note on the Game of Life in
 * Hexagonal and Pentagonal Tessellations," Complex Systems 15 (2005).
 *
 * §1 config  §2 state  §3 performance  §4 color  §5 logic  §6 simulation
 * §7 effects  §8 seed/reset  §9 render  §10 platform  §11 driver
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — constants, colour-pair ids, theme palette table ── */

#define GRID_W_MAX  200
#define GRID_H_MAX   60
#define HUD_TOP_ROWS  2   /* top rows: stats + colour legend           */
#define HUD_BOT_ROWS  1   /* bottom row: key hints                     */
#define HUD_ROWS      (HUD_TOP_ROWS + HUD_BOT_ROWS)   /* rows the HUD reserves */

#define RENDER_NS    (1000000000LL / 60)   /* one frame's length at 60 fps      */
#define GEN_HZ_DEF    8     /* starting speed, generations per second            */
#define GEN_HZ_MIN    2
#define GEN_HZ_MAX   30
#define DT_CAP_NS    (100 * 1000000LL)      /* longest frame gap we'll honour, so a stall can't make the sim sprint */
#define GLOW_RATE    0.28f /* how fast a cell's glow drifts toward its target each frame */
#define GLOW_GONE    0.10f /* glow this faint counts as fully gone — back to the dim lattice dot */
#define NS_PER_SEC   1000000000.0  /* nanoseconds in a second, for the timing maths */
#define FPS_SMOOTH   0.1f          /* how much each new reading nudges the shown fps */
#define DENSITY_DEFAULT  35        /* default fill density, percent of cells alive */

/* ncurses colour-pair slots.  CP_HEAT0..7 are the eight colours a cell can
 * take based on how old it is — newborn (hottest) down to settled-old
 * (coolest).  The slots stay fixed but their actual colours come from whichever
 * theme is active, so pressing 't' recolours the whole board without touching
 * any draw code.  CP_HEAT0..7 MUST stay consecutive: theme_apply walks them as
 * CP_HEAT0 + i, and each cell remembers one of these ids.
 *
 * The age ranges below are deliberately uneven — tight early, loose later — so a
 * cell visibly cools over its first few generations, then holds a steady colour
 * once it's part of a stable shape (even buckets would either change too slowly
 * when young or keep flickering when old). */
enum {
    CP_DEAD = 1,   /* dead     — dim dot, outlines the hex lattice */
    CP_HEAT0,      /* age 0      newborn (hottest)   */
    CP_HEAT1,      /* age 1                          */
    CP_HEAT2,      /* age 2-3                        */
    CP_HEAT3,      /* age 4-6                        */
    CP_HEAT4,      /* age 7-10                       */
    CP_HEAT5,      /* age 11-16                      */
    CP_HEAT6,      /* age 17-25                      */
    CP_HEAT7,      /* age 26+    settled (coolest)   */
    CP_HUD,        /* HUD data    — bright yellow on default bg */
    CP_HINT,       /* HUD actions — bright cyan   on default bg */
};

/* Theme — one named colour gradient for the age ramp: 8 colours running hot
 * (newborn) down to cool (old), one per heat tier CP_HEAT0..7.  Turning a number
 * (here a cell's age) into a colour gradient is the standard way to show data
 * with colour — Moreland, "Diverging Color Maps for Scientific Visualization,"
 * Proc. ISVC 2009.  Pressing 't' just re-fills those 8 ncurses pairs, so the
 * whole board recolours instantly without touching any draw code.
 *
 * Every 256-colour stop is kept in the bright half of the palette (>= 24, or
 * >= 240 on the grey ramp) so even the dimmest tier stays readable under A_DIM
 * — the project rule in COLOR.md, since the dark end of the palette shows up as
 * near-black.  ramp8 is the plain 8-colour version used on terminals that can't
 * do 256 colours. */
typedef struct {
    short ramp[8];     /* the 8 colours, index 0 = newborn (hot) .. 7 = oldest (cool) */
    short ramp8[8];    /* same 8 in the 8-colour fallback, same hot-to-cool order     */
    const char *name;  /* shown in the HUD                                            */
} Theme;

#define N_THEMES 5
static const Theme k_themes[N_THEMES] = {
    /* 0  Ember   — white-hot young cooling to deep blue (the classic) */
    { { 231, 226, 220, 214, 208,  51,  38,  26 },
      { COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED,
        COLOR_CYAN,  COLOR_CYAN,   COLOR_BLUE }, "Ember" },
    /* 1  Inferno — pure fire: white-hot down to dark red */
    { { 231, 228, 220, 214, 208, 202, 196, 124 },
      { COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED,
        COLOR_RED,   COLOR_RED,    COLOR_RED },  "Inferno" },
    /* 2  Ocean   — white spray through cyan to deep blue */
    { { 231, 195, 159,  87,  51,  39,  33,  26 },
      { COLOR_WHITE, COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN, COLOR_CYAN,
        COLOR_BLUE,  COLOR_BLUE,   COLOR_BLUE }, "Ocean" },
    /* 3  Forest  — pale shoots cooling to deep evergreen */
    { { 231, 230, 191, 154, 118,  78,  35,  28 },
      { COLOR_WHITE, COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN,  COLOR_GREEN }, "Forest" },
    /* 4  Plasma  — white through magenta and violet to blue */
    { { 231, 219, 213, 207, 171, 135,  99,  57 },
      { COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_BLUE,  COLOR_BLUE,    COLOR_BLUE }, "Plasma" },
};

/* ── §2 state — the data types and the one Scene that holds them ── */

/* Colony — the living board: one generation of cells on the hex grid, plus
 * everything needed to step it forward.  This is Game of Life on a 6-neighbour
 * hex grid under rule B2/S34 (a dead cell is born on exactly 2 live neighbours;
 * a live cell survives on 3 or 4).  The hex tiling rule comes from Bays, Complex
 * Systems 15 (2005).
 *
 * Why two copies of the board (grid/next, age/anext): a step has to build the
 * whole next generation from a frozen picture of the current one — every cell
 * counts its neighbours as they are right now.  If we wrote changes in place, a
 * cell could see neighbours that already updated this tick and the rule would
 * break.  So we fill next/anext, then copy them back over grid/age.
 *
 * What "age" is: how many generations a cell has been alive without a break
 * (survivors count up, newborns restart at 0).  It only drives the colour — it
 * never affects whether a cell lives or dies. */
typedef struct {
    signed char   grid [GRID_H_MAX][GRID_W_MAX];  /* the real board — 0 dead, 1 alive                         */
    signed char   next [GRID_H_MAX][GRID_W_MAX];  /* scratch for the next generation, copied back over grid   */
    unsigned char age  [GRID_H_MAX][GRID_W_MAX];  /* generations each cell has been alive, 0..255 — sets colour */
    unsigned char anext[GRID_H_MAX][GRID_W_MAX];  /* scratch ages for the next generation, paired with next   */
    int           gh, gw;                          /* board size in cells right now (fits the terminal, capped at _MAX) */
    long long     gen;                             /* generations since the last reset, shown in the HUD       */
} Colony;

/* GlowField — a smoothing layer that softens the on/off blink, one value per
 * cell.  Cells snap fully on or off each generation, which the eye reads as
 * flicker.  The fix: give each cell a brightness that drifts toward its target
 * over a few frames instead of jumping, so a birth swells in and a death fades
 * out.  Purely cosmetic — the B2/S34 rule never looks at it.
 *
 * The brightness tracks how STEADY a cell has been, not how old it is: only
 * cells that have held still a while reach full glow, so the churning edges of a
 * growing pattern stay dim and quiet while settled shapes read as bright dots. */
typedef struct {
    float         glow[GRID_H_MAX][GRID_W_MAX];   /* brightness 0..1, drifting toward alive=1 / dead=0; sets dot size and intensity   */
    unsigned char fade[GRID_H_MAX][GRID_W_MAX];   /* the colour a cell had while alive, so it fades out in its own colour; stored as a pair id (not a raw colour) so theme swaps recolour it for free */
} GlowField;

/* Scene — the whole program's state gathered in one place.  Frame-level
 * functions take the whole Scene; the small workers below take only the piece
 * (Colony or GlowField) they actually need, so keeping the data together doesn't
 * tangle the simulation, effects and drawing back into each other. */
typedef struct {
    Colony    colony;   /* the living board                                         */
    GlowField glow;     /* the smoothing layer drawn over it                        */
    int       speed;    /* generations per second, kept in GEN_HZ_MIN..GEN_HZ_MAX   */
    bool      paused;   /* freeze the sim (drawing and the glow fade keep running)  */
    int       theme;    /* which palette is active (index into k_themes)            */
} Scene;

/* The one program instance.  The big per-cell arrays live inside it, so this is
 * a single static object (no heap — the project allocates nothing after init).
 * speed is the only field that starts non-zero. */
static Scene g_scene = { .speed = GEN_HZ_DEF };

/* Terminal size, HUD stats, and signal flags — small side data that would just
 * clutter the Scene, so they stay as plain globals. */
static int       g_rows, g_cols;                /* terminal size, in characters    */
static long long g_live = 0;                    /* live-cell count, recounted each frame */
static double    g_fps  = 60.0;                 /* smoothed frames per second        */
static bool      g_has_256 = false;             /* does the terminal do 256 colours? */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ── §3 performance — a clock to read time and a sleep to cap the frame rate ── */

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

/* ── §4 color — set up the palette; the only place colours are loaded ── */

/* theme_apply — load the 8 age colours from theme ti.  Because it reuses the
 * same pair slots, switching themes recolours every cell at once with no other
 * work.  The dead/HUD/hint colours don't change with the theme. */
static void theme_apply(int ti)
{
    const Theme *th = &k_themes[ti];
    for (int i = 0; i < 8; i++)
        init_pair(CP_HEAT0 + i, g_has_256 ? th->ramp[i] : th->ramp8[i], -1);
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_has_256 = (COLORS >= 256);

    /* these don't depend on the theme — set once and left alone */
    if (g_has_256) {
        init_pair(CP_DEAD, 242, -1);   /* dim lattice dot         */
        init_pair(CP_HUD,  226, -1);   /* bright yellow — data    */
        init_pair(CP_HINT,  51, -1);   /* bright cyan   — actions */
    } else {
        init_pair(CP_DEAD, COLOR_BLACK,  -1);
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(g_scene.theme);   /* load the age colours from the active theme */
}

/* ── §5 logic — small pure decisions: age → colour, glow → glyph ── */

/* heat_pair — pick a live cell's colour from its age.  Young cells run hot
 * (white, gold, orange); old cells run cool (cyan, teal, deep blue).  Colour is
 * the only thing age changes — the shape stays the same so cells don't pop. */
static int heat_pair(unsigned char age)
{
    if      (age == 0)  return CP_HEAT0;   /* white-hot */
    else if (age <= 1)  return CP_HEAT1;   /* gold      */
    else if (age <= 3)  return CP_HEAT2;   /* amber     */
    else if (age <= 6)  return CP_HEAT3;   /* orange    */
    else if (age <= 10) return CP_HEAT4;   /* ember     */
    else if (age <= 16) return CP_HEAT5;   /* cyan      */
    else if (age <= 25) return CP_HEAT6;   /* teal      */
    else                return CP_HEAT7;   /* deep blue */
}

/* glow_glyph — turn a cell's glow into the dot to draw.  A cell that's just born
 * or dying is a faint speck '.'; a settled cell grows into a round 'o', bold at
 * full glow.  Since glow drifts smoothly, the dot grows and shrinks instead of
 * snapping, and the busy growing edges stay small and quiet. */
static void glow_glyph(float gl, char *ch, attr_t *at)
{
    if      (gl >= 0.55f) { *ch = 'o'; *at = A_BOLD;   }  /* settled — bright */
    else if (gl >= 0.25f) { *ch = 'o'; *at = A_NORMAL; }  /* mid              */
    else                  { *ch = '.'; *at = A_DIM;    }  /* faint speck      */
}

/* ── §6 simulation — step the board one generation, plus the neighbour count ── */

/* The six neighbour directions.  The row offsets are shared, but the column
 * offsets differ between even and odd rows because of the hex stagger. */
static const int HEX_DR[6]    = { -1, -1,  0, 0,  1,  1 };
static const int HEX_DC_EVEN[6] = { -1,  0, -1, 1, -1,  0 };
static const int HEX_DC_ODD[6]  = {  0,  1, -1, 1,  0,  1 };

/* hex_count — count the live neighbours of cell (r,c).  Off-board counts as dead
 * (the board has hard edges — patterns don't wrap around). */
static int hex_count(const Colony *col, int r, int c)
{
    const int *dc = (r & 1) ? HEX_DC_ODD : HEX_DC_EVEN;
    int cnt = 0;
    for (int k = 0; k < 6; k++) {
        int nr = r + HEX_DR[k];
        int nc = c + dc[k];
        if (nr >= 0 && nr < col->gh && nc >= 0 && nc < col->gw)
            cnt += (col->grid[nr][nc] != 0) ? 1 : 0;
    }
    return cnt;
}

/* colony_step — advance the board one generation: work out every cell's next
 * state into the scratch copies, then copy them back.  The only function that
 * changes the simulation. */
static void colony_step(Colony *col)
{
    for (int r = 0; r < col->gh; r++) {
        for (int c = 0; c < col->gw; c++) {
            int n = hex_count(col, r, c);
            int alive = (col->grid[r][c] != 0);
            /* B2/S34: a live cell survives on 3 or 4, a dead cell is born on 2 */
            if (alive && (n == 3 || n == 4)) {
                col->next[r][c]  = 1;
                /* one generation older, but never past 255 */
                col->anext[r][c] = col->age[r][c] < 255 ? col->age[r][c] + 1 : 255;
            } else if (!alive && n == 2) {
                col->next[r][c]  = 1;
                col->anext[r][c] = 0;   /* just born */
            } else {
                col->next[r][c]  = 0;
                col->anext[r][c] = 0;
            }
        }
    }
    /* make the new generation the real one and bump the counter */
    col->gen++;
    memcpy(col->grid, col->next,  sizeof col->grid);
    memcpy(col->age,  col->anext, sizeof col->age);
}

/* ── §7 effects — advance the glow/fade smoothing (cosmetic only) ── */

/* glow_sync — set every cell's glow straight to alive=1 / dead=0 with no fade.
 * Used after a reseed or clear so the new board shows up right away. */
static void glow_sync(const Colony *col, GlowField *gf)
{
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++)
            gf->glow[r][c] = col->grid[r][c] ? 1.0f : 0.0f;
}

/* ease_toward — nudge `cur` a little closer to `target`, closing a fixed
 * fraction (GLOW_RATE) of the gap each call.  That's what makes glow drift
 * smoothly instead of jumping. */
static float ease_toward(float cur, float target)
{
    return cur + (target - cur) * GLOW_RATE;
}

/* glow_step — move every cell's brightness one frame toward its target (alive
 * heads to 1, dead heads to 0), and remember each live cell's colour so it can
 * fade out in that colour later.  Runs every frame, not every generation. */
static void glow_step(const Colony *col, GlowField *gf)
{
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++) {
            if (col->grid[r][c]) {
                gf->glow[r][c] = ease_toward(gf->glow[r][c], 1.0f);
                gf->fade[r][c] = (unsigned char)heat_pair(col->age[r][c]);
            } else {
                gf->glow[r][c] = ease_toward(gf->glow[r][c], 0.0f);
            }
        }
}

/* ── §8 seed/reset — fill or clear the board on a key press (not a tick) ── */

/* Each of these rewrites the board, then snaps the glow layer to match so the
 * new board shows up at once instead of fading in. */

static void colony_random(Colony *col, GlowField *gf, int density_pct)
{
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++)
            col->grid[r][c] = (rand() % 100 < density_pct) ? 1 : 0;
    memset(col->age, 0, sizeof col->age);
    col->gen = 0;
    glow_sync(col, gf);
}

static void colony_clear(Colony *col, GlowField *gf)
{
    memset(col->grid, 0, sizeof col->grid);
    memset(col->age,  0, sizeof col->age);
    col->gen = 0;
    glow_sync(col, gf);
}

/* colony_seed — drop a small random blob in the middle of an empty board. */
static void colony_seed(Colony *col, GlowField *gf)
{
    colony_clear(col, gf);
    int cr = col->gh / 2, cc = col->gw / 2;
    /* a small dense cluster of randomly on/off cells */
    for (int dr = -3; dr <= 3; dr++)
        for (int dc = -3; dc <= 3; dc++) {
            int r = cr+dr, c = cc+dc;
            if (r>=0&&r<col->gh&&c>=0&&c<col->gw)
                col->grid[r][c] = rand()%2;
        }
    glow_sync(col, gf);   /* clear left glow at 0; resync now that cells are placed */
}

/* ── §9 render — draw the board and the HUD; only reads state ── */

/* count_live — count how many cells are alive, for the HUD. */
static long long count_live(const Colony *col)
{
    long long live = 0;
    for (int r = 0; r < col->gh; r++)
        for (int c = 0; c < col->gw; c++)
            if (col->grid[r][c]) live++;
    return live;
}

/* hud_line — print one HUD line, cut off at the last column so a long line never
 * spills onto the grid. */
static void hud_line(int row, int pair, attr_t attr, const char *s)
{
    attron(COLOR_PAIR(pair) | attr);
    mvaddnstr(row, 1, s, g_cols - 1);
    attroff(COLOR_PAIR(pair) | attr);
}

/* draw_colony — paint the grid of dots.  Each cell gets a 2-column slot but
 * draws a single dot, and odd rows shift one column right to give the hex
 * stagger.  The dot's size comes from glow ('.' to 'o'), its colour from age; a
 * fully dead cell is just the dim '.' lattice underneath. */
static void draw_colony(const Scene *s)
{
    const Colony    *col = &s->colony;
    const GlowField *gf  = &s->glow;

    for (int r = 0; r < col->gh && r + HUD_TOP_ROWS < g_rows - HUD_BOT_ROWS; r++) {
        int offset = r & 1;   /* odd rows shift right one column for the hex stagger */
        for (int c = 0; c < col->gw; c++) {
            int sc = c * 2 + offset;
            int sr = r + HUD_TOP_ROWS;
            if (sc >= g_cols) break;

            float  gl = gf->glow[r][c];
            int    cp;
            attr_t at;
            char   ch;

            if (col->grid[r][c] || gl > GLOW_GONE) {
                /* alive, or still fading out: draw a dot sized by glow and
                 * coloured by the cell's colour (its last colour if it's dying),
                 * so the change is smooth */
                cp = gf->fade[r][c];
                glow_glyph(gl, &ch, &at);
            } else {
                /* fully dead: the dim '.' lattice */
                cp = CP_DEAD; ch = '.'; at = A_DIM;
            }
            attron(COLOR_PAIR(cp) | at);
            mvaddch(sr, sc, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp) | at);
        }
    }
}

/* draw_hud — the two top rows (stats and colour legend) and the bottom row of
 * key hints. */
static void draw_hud(const Scene *s)
{
    char buf[160];
    snprintf(buf, sizeof buf,
        "HexLife B2/S34  %.1f fps  gen:%lld  live:%lld  gen/s:%d  [%s]  %s",
        g_fps, s->colony.gen, g_live, s->speed,
        k_themes[s->theme].name, s->paused ? "PAUSED" : "running");
    hud_line(0, CP_HUD, A_BOLD, buf);   /* main stats — bold so it stands out */
    hud_line(1, CP_HUD, 0,              /* colour legend — same yellow, not bold */
        "o=live dot (colour=age: white-hot new .. deep-blue old)   .=dead");
    hud_line(g_rows - 1, CP_HINT, A_BOLD,
        "q:quit  spc:random  1-5:density  r:seed  c:clear  t:theme  p:pause  +/-:speed");
}

/* scene_draw — one frame: the board first, then the HUD on top. */
static void scene_draw(const Scene *s)
{
    draw_colony(s);
    draw_hud(s);
}


/* ── §10 platform — signals, random seed, terminal setup and cleanup ── */

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void seed_rng(void) { srand((unsigned)time(NULL)); }

static void install_signals(void)
{
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);
}

/* screen_init — set the terminal up for full-screen animation: raw keys, no
 * echo, no cursor, getch doesn't block.  typeahead(-1) keeps incoming keypresses
 * from interrupting a draw and tearing the screen. */
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
}

/* ── §11 driver — setup, input, resize, and the main loop ── */

/* fit_to_terminal — size the board to the current terminal.  Each cell is 2
 * columns wide with a 1-column margin, so (cols-2)/2 fit across; the HUD rows
 * eat into the height. */
static void fit_to_terminal(Colony *col)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    col->gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
    col->gw = ((cols - 2) / 2) < GRID_W_MAX ? ((cols - 2) / 2) : GRID_W_MAX;
}

/* handle_resize — terminal changed size: pick up the new size and refill the
 * board (endwin/refresh is how ncurses learns the new dimensions). */
static void handle_resize(Scene *s)
{
    endwin(); refresh();
    fit_to_terminal(&s->colony);
    colony_random(&s->colony, &s->glow, DENSITY_DEFAULT);
}

/* handle_key — do whatever one keypress asks: quit, refill or seed the board,
 * change theme, pause, or change the speed. */
static void handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1; break;
    case ' ': colony_random(&s->colony, &s->glow, DENSITY_DEFAULT); break;
    case 'p': case 'P': s->paused = !s->paused; break;
    case 'c': case 'C': colony_clear(&s->colony, &s->glow); break;
    case 'r': case 'R': colony_seed(&s->colony, &s->glow); break;
    case 't': s->theme = (s->theme + 1) % N_THEMES; theme_apply(s->theme); break;
    case 'T': s->theme = (s->theme + N_THEMES - 1) % N_THEMES; theme_apply(s->theme); break;
    case '1': colony_random(&s->colony, &s->glow, 20); break;
    case '2': colony_random(&s->colony, &s->glow, DENSITY_DEFAULT); break;
    case '3': colony_random(&s->colony, &s->glow, 50); break;
    case '4': colony_random(&s->colony, &s->glow, 65); break;
    case '5': colony_random(&s->colony, &s->glow, 80); break;
    case '+': case '=': if (s->speed < GEN_HZ_MAX) s->speed++; break;
    case '-':           if (s->speed > GEN_HZ_MIN) s->speed--; break;
    default: break;
    }
}

/* advance_sim — keep the simulation in step with real time, separately from the
 * frame rate: bank the time that has passed and run one generation for each
 * 1/speed second of it, so the screen stays smooth at any speed. */
static void advance_sim(Scene *s, double *accum, double dt)
{
    if (s->paused) return;
    double ns_per_gen = NS_PER_SEC / (double)s->speed;
    *accum += dt;
    while (*accum >= ns_per_gen) { colony_step(&s->colony); *accum -= ns_per_gen; }
}

int main(void)
{
    Scene *s = &g_scene;

    /* setup */
    seed_rng();
    atexit(cleanup);
    install_signals();
    screen_init();
    color_init();
    fit_to_terminal(&s->colony);
    colony_random(&s->colony, &s->glow, DENSITY_DEFAULT);

    long long prev  = clock_ns();
    double    accum = 0.0;   /* time banked up toward the next generation */

    while (!g_quit) {
        if (g_resize) { g_resize = 0; handle_resize(s); }
        handle_key(s, getch());

        /* frame timing: measure how long the last frame took, smooth the fps
         * readout, and cap the gap so a long stall can't make the sim sprint */
        long long now = clock_ns();
        double    dt  = (double)(now - prev);
        prev = now;
        if (dt > 0) g_fps += (NS_PER_SEC / dt - g_fps) * FPS_SMOOTH;
        if (dt > (double)DT_CAP_NS) dt = (double)DT_CAP_NS;

        advance_sim(s, &accum, dt);

        /* draw one frame */
        g_live = count_live(&s->colony);
        glow_step(&s->colony, &s->glow);   /* nudge each cell's glow toward the new board */
        erase();
        scene_draw(s);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));   /* wait out the rest of the 60 fps frame */
    }
    return 0;
}
