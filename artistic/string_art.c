/*
 * string_art.c — Mathematical String Art
 *
 * N nails evenly spaced on a circle; threads connect nail i to nail
 * round(i × k) mod N.  k drifts slowly through [2, 7.5]:
 *
 *   k ≈ 2  →  Cardioid       (1 cusp)
 *   k ≈ 3  →  Nephroid       (2 cusps)
 *   k ≈ 4  →  Deltoid        (3 cusps)
 *   k ≈ 5  →  Astroid        (4 cusps)
 *   k ≈ 6  →  5-Epicycloid
 *   k ≈ 7  →  6-Epicycloid
 *
 * Three thread sets (phase-offset by 0.18) drawn in separate colours;
 * where sets overlap the cell brightens to white.  A float density
 * buffer fades each frame — threads linger like long-exposure photography.
 *
 * Keys:
 *   q / Q     quit
 *   space     pause / resume
 *   r / R     reset  (k → 2, clear density)
 *   + / =     increase k speed  × 1.5
 *   - / _     decrease k speed  ÷ 1.5
 *   f         decrease fade  (trails linger longer)
 *   F         increase fade  (trails clear faster)
 *   t / T     next colour theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/string_art.c \
 *       -o string_art -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 coords  §5 sim
 *           §6 scene   §7 screen §8 app
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS       33333333LL  /* ~30 fps                              */
#define N_NAILS       200         /* nails on circle                      */
#define K_START       2.0f        /* lowest k (cardioid)                  */
#define K_END         7.5f        /* highest k before wrap                */
#define K_SPEED_DEF   0.0015f     /* k increment per tick                 */
#define K_SPEED_MIN   0.0002f
#define K_SPEED_MAX   0.020f
#define N_SETS        3           /* overlapping thread sets              */
#define FADE_DEF      0.900f      /* density multiplied by this each tick */
#define FADE_MIN      0.700f
#define FADE_MAX      0.995f
#define ADD_PER_HIT   0.050f      /* density added per Bresenham pixel    */
#define CELL_W        8           /* pixel sub-cells per terminal column  */
#define CELL_H        16          /* pixel sub-cells per terminal row     */
#define MAX_ROWS      128
#define MAX_COLS      320

/* phase offsets between the three sets */
static const float K_OFF[N_SETS] = { 0.00f, 0.18f, 0.36f };

/* color-pair IDs */
enum { CP_BG = 1, CP_S0, CP_S1, CP_S2, CP_BRIGHT, CP_HUD };

/* 4 themes: [theme][set_index] → 256-color fg  (8-color fallback below) */
static const short THEMES[4][N_SETS] = {
    {  51, 213, 226 },   /* cyan / magenta / yellow  — classic */
    { 196, 208, 220 },   /* red  / orange  / gold    — fire    */
    {  45,  75, 123 },   /* sky  / indigo  / navy    — ice     */
    {  82, 201,  46 },   /* lime / violet  / green   — neon    */
};
static const char * const THEME_NAMES[] = { "Classic", "Fire", "Ice", "Neon" };
#define N_THEMES 4

/* density-to-character ramp (low → high density) */
static const char RAMP[] = " .:-=+*#@";
#define RAMP_N ((int)(sizeof(RAMP) - 1))

static const char *shape_name(float k)
{
    switch ((int)(k + 0.15f)) {
        case 2:  return "Cardioid";
        case 3:  return "Nephroid";
        case 4:  return "Deltoid";
        case 5:  return "Astroid";
        case 6:  return "5-Epicycloid";
        case 7:  return "6-Epicycloid";
        default: return "Morphing";
    }
}

/* ── §2 clock ────────────────────────────────────────────────────────── */

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

/* ── §3 color ────────────────────────────────────────────────────────── */

static int g_theme = 0;

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(CP_BG,     -1,                                  -1);
    init_pair(CP_BRIGHT, (COLORS >= 256) ? 231 : COLOR_WHITE, -1);
    init_pair(CP_HUD,    (COLORS >= 256) ?  82 : COLOR_GREEN, -1);
}

static void theme_apply(void)
{
    if (COLORS >= 256) {
        init_pair(CP_S0, THEMES[g_theme][0], -1);
        init_pair(CP_S1, THEMES[g_theme][1], -1);
        init_pair(CP_S2, THEMES[g_theme][2], -1);
    } else {
        static const short fb[N_SETS] =
            { COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW };
        init_pair(CP_S0, fb[0], -1);
        init_pair(CP_S1, fb[1], -1);
        init_pair(CP_S2, fb[2], -1);
    }
}

/* ── §4 coords ───────────────────────────────────────────────────────── */

static int g_nail_cx[N_NAILS];   /* cell-space nail positions */
static int g_nail_cy[N_NAILS];
static int g_rows, g_cols;

static void compute_nails(int rows, int cols)
{
    g_rows = rows;
    g_cols = cols;
    /* pixel-space circle → maps to an on-screen circle (aspect-correct) */
    float r_px  = fminf((float)cols * CELL_W, (float)rows * CELL_H) * 0.44f;
    float cx_px = (float)cols * CELL_W * 0.5f;
    float cy_px = (float)rows * CELL_H * 0.5f;
    for (int i = 0; i < N_NAILS; i++) {
        float a  = 2.0f * (float)M_PI * i / N_NAILS;
        int   cx = (int)(cx_px + r_px * cosf(a)) / CELL_W;
        int   cy = (int)(cy_px + r_px * sinf(a)) / CELL_H;
        g_nail_cx[i] = cx < 0 ? 0 : cx >= cols ? cols - 1 : cx;
        g_nail_cy[i] = cy < 0 ? 0 : cy >= rows ? rows - 1 : cy;
    }
}

/* ── §5 simulation ───────────────────────────────────────────────────── */

/* per-set float density grid; values in [0, 1] */
static float g_dens[N_SETS][MAX_ROWS * MAX_COLS];

#define DIDX(r, c)  ((r) * g_cols + (c))

/*
 * Add 'val' to every cell on the Bresenham segment (x0,y0)→(x1,y1).
 * Unsigned cast trick: negative coords → large unsigned → fails < check.
 */
static void bres_add(float *buf, int x0, int y0, int x1, int y1, float val)
{
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if ((unsigned)x0 < (unsigned)g_cols &&
            (unsigned)y0 < (unsigned)g_rows) {
            float *p = &buf[DIDX(y0, x0)];
            *p += val;
            if (*p > 1.0f) *p = 1.0f;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static float g_k;
static float g_k_speed;
static float g_fade;
static int   g_paused;

static float wrap_k(float k)
{
    float range = K_END - K_START;
    while (k >= K_END)   k -= range;
    while (k <  K_START) k += range;
    return k;
}

static void sim_init(void)
{
    g_k       = K_START;
    g_k_speed = K_SPEED_DEF;
    g_fade    = FADE_DEF;
    g_paused  = 0;
    memset(g_dens, 0, sizeof(g_dens));
}

static void sim_tick(void)
{
    if (g_paused) return;

    g_k = wrap_k(g_k + g_k_speed);

    /* fade all density buffers */
    int total = g_rows * g_cols;
    for (int s = 0; s < N_SETS; s++)
        for (int i = 0; i < total; i++)
            g_dens[s][i] *= g_fade;

    /* draw threads for each set */
    for (int s = 0; s < N_SETS; s++) {
        float k = wrap_k(g_k + K_OFF[s]);
        for (int i = 0; i < N_NAILS; i++) {
            /* roundf gives smoother morphing between integer-k shapes */
            int j = (int)roundf((float)i * k) % N_NAILS;
            if (j < 0) j += N_NAILS;
            bres_add(g_dens[s],
                     g_nail_cx[i], g_nail_cy[i],
                     g_nail_cx[j], g_nail_cy[j],
                     ADD_PER_HIT);
        }
    }
}

/* ── §6 scene ────────────────────────────────────────────────────────── */

static const int SET_CP[N_SETS] = { CP_S0, CP_S1, CP_S2 };

static void scene_draw(void)
{
    for (int r = 0; r < g_rows; r++) {
        for (int c = 0; c < g_cols - 1; c++) {
            float d[N_SETS];
            float dmax = 0.0f;
            int   smax = 0;
            int   n_hot = 0;

            for (int s = 0; s < N_SETS; s++) {
                d[s] = g_dens[s][DIDX(r, c)];
                if (d[s] > dmax) { dmax = d[s]; smax = s; }
            }

            if (dmax < 0.015f) { mvaddch(r, c, ' '); continue; }

            /* cell is "hot" for a set if its density is ≥ 45% of the max */
            for (int s = 0; s < N_SETS; s++)
                if (d[s] >= dmax * 0.45f) n_hot++;

            /* overlap of 2+ sets → white; single-set → that set's colour */
            int cp = (n_hot > 1) ? CP_BRIGHT : SET_CP[smax];
            int ci = (int)(dmax * (float)(RAMP_N - 1) + 0.5f);
            if (ci >= RAMP_N) ci = RAMP_N - 1;

            attron(COLOR_PAIR(cp));
            mvaddch(r, c, (chtype)(unsigned char)RAMP[ci]);
            attroff(COLOR_PAIR(cp));
        }
    }
}

static void scene_hud(void)
{
    if (g_rows < 2) return;
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " k=%.3f [%-12s] spd=%.4f fade=%.2f %s theme=%-7s"
             "  spc:pause r:reset +/-:spd f/F:fade t:theme q:quit ",
             (double)g_k, shape_name(g_k),
             (double)g_k_speed, (double)g_fade,
             g_paused ? "PAUSED" : "      ",
             THEME_NAMES[g_theme]);
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §7 screen ───────────────────────────────────────────────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    theme_apply();
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    compute_nails(rows, cols);
    memset(g_dens, 0, sizeof(g_dens));
    erase();
}

/* ── §8 app ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running     = 0;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    screen_init();
    screen_resize();
    sim_init();

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin();
            refresh();
            screen_resize();
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0;  break;
            case ' ':           g_paused ^= 1;  break;
            case 'r': case 'R':
                memset(g_dens, 0, sizeof(g_dens));
                g_k = K_START;
                break;
            case '+': case '=':
                g_k_speed *= 1.5f;
                if (g_k_speed > K_SPEED_MAX) g_k_speed = K_SPEED_MAX;
                break;
            case '-': case '_':
                g_k_speed /= 1.5f;
                if (g_k_speed < K_SPEED_MIN) g_k_speed = K_SPEED_MIN;
                break;
            case 'f':
                g_fade -= 0.02f;
                if (g_fade < FADE_MIN) g_fade = FADE_MIN;
                break;
            case 'F':
                g_fade += 0.02f;
                if (g_fade > FADE_MAX) g_fade = FADE_MAX;
                break;
            case 't': case 'T':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply();
                break;
            }
        }

        sim_tick();

        erase();
        scene_draw();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }

    return 0;
}
