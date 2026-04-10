/*
 * cellular_automata_1d.c — Wolfram 1-D Elementary Cellular Automaton
 *
 * The pattern builds row-by-row from the top of the screen downward.
 * When the screen fills it holds for 3 seconds then loads the next preset.
 *
 * Layout:
 *   Row 0          — title bar: rule number, name, class (in class colour)
 *   Rows 1 … n-2   — CA area, filling top-down one row at a time
 *   Row n-1        — key-binding strip
 *
 * Each rule has a Wolfram class, colour-coded:
 *   Class 1 (Fixed)    — grey    — converges to uniform state
 *   Class 2 (Periodic) — cyan    — stable repeating patterns
 *   Class 3 (Chaotic)  — orange  — random-looking, sensitive to seed
 *   Class 4 (Complex)  — green   — localised glider-like structures
 *   Class 5 (Fractal)  — yellow  — Sierpinski / self-similar triangles
 *
 * Keys:
 *   n / p         next / previous preset
 *   a             toggle auto-advance (on by default)
 *   r             reseed — single cell at centre
 *   R             reseed — random initial row
 *   + / =         faster (fewer ticks between rows)
 *   - / _         slower (more ticks between rows)
 *   space         pause / resume
 *   0-9 + Enter   type any rule 0–255 and press Enter to apply
 *   Backspace     erase last typed digit
 *   q / Q         quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/cellular_automata_1d.c \
 *       -o cellular_automata_1d -lncurses
 *
 * Sections: §1 config  §2 clock  §3 color  §4 ca  §5 scene
 *           §6 screen  §7 app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS      33333333LL  /* ~30 fps                               */
#define MAX_ROWS     128         /* grid depth; must be ≥ terminal rows   */
#define MAX_COLS     320
#define DELAY_DEF    3           /* ticks between rows  (~10 rows / sec)  */
#define DELAY_MIN    1           /* fastest: 1 tick / row  (30 rows/sec)  */
#define DELAY_MAX    30          /* slowest: 30 ticks / row (1 row/sec)   */
#define PAUSE_TICKS  90          /* hold complete pattern for 3 s then advance */
#define LIVE_CHAR    '#'

/* simulation states */
enum { ST_BUILD = 0, ST_PAUSE };

/* color-pair IDs */
enum { CP_CL1=1, CP_CL2, CP_CL3, CP_CL4, CP_CL5, CP_HUD };

/* Preset bank — visually striking rules */
#define N_PRESETS 17
static const struct { int rule; const char *desc; } PRESETS[N_PRESETS] = {
    {  30, "Chaos / RNG"       },
    {  90, "Sierpinski"        },
    { 110, "Turing-complete"   },
    {  18, "Sierpinski-like"   },
    { 150, "Pascal mod 2"      },
    {  60, "XOR fractal"       },
    {  54, "Complex"           },
    { 105, "Complex / fractal" },
    { 106, "Complex"           },
    {  45, "Chaotic"           },
    {  22, "Chaotic"           },
    { 126, "Chaotic"           },
    {  57, "Complex"           },
    {  73, "Complex"           },
    {  99, "Complex"           },
    {   0, "All zeros"         },
    { 255, "All ones"          },
};

/* Wolfram class: 1=fixed 2=periodic 3=chaotic 4=complex 5=fractal */
static int ca_classify(int r)
{
    static const int cl5[] = { 18, 60, 90, 105, 150 };
    static const int cl4[] = { 54, 57, 62, 73, 99, 106, 110 };
    static const int cl3[] = { 22, 30, 45, 75, 89, 109, 126, 135, 149, 153, 154 };
    static const int cl1[] = { 0, 8, 32, 40, 128, 136, 160, 168, 255 };
    for (int i = 0; i < 5;  i++) if (cl5[i] == r) return 5;
    for (int i = 0; i < 7;  i++) if (cl4[i] == r) return 4;
    for (int i = 0; i < 11; i++) if (cl3[i] == r) return 3;
    for (int i = 0; i < 9;  i++) if (cl1[i] == r) return 1;
    return 2;
}

static const char *class_name(int cls)
{
    switch (cls) {
        case 1: return "Fixed";
        case 2: return "Periodic";
        case 3: return "Chaotic";
        case 4: return "Complex";
        case 5: return "Fractal";
    }
    return "Unknown";
}

static int class_cp(int cls)
{
    switch (cls) {
        case 1: return CP_CL1;
        case 2: return CP_CL2;
        case 3: return CP_CL3;
        case 4: return CP_CL4;
        case 5: return CP_CL5;
    }
    return CP_CL2;
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

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_CL1, 244, -1);  /* grey   — fixed    */
        init_pair(CP_CL2,  51, -1);  /* cyan   — periodic */
        init_pair(CP_CL3, 202, -1);  /* orange — chaotic  */
        init_pair(CP_CL4,  82, -1);  /* green  — complex  */
        init_pair(CP_CL5, 226, -1);  /* yellow — fractal  */
        init_pair(CP_HUD,  82, -1);
    } else {
        init_pair(CP_CL1, COLOR_WHITE,   -1);
        init_pair(CP_CL2, COLOR_CYAN,    -1);
        init_pair(CP_CL3, COLOR_RED,     -1);
        init_pair(CP_CL4, COLOR_GREEN,   -1);
        init_pair(CP_CL5, COLOR_YELLOW,  -1);
        init_pair(CP_HUD, COLOR_GREEN,   -1);
    }
}

/* ── §4 CA ───────────────────────────────────────────────────────────── */

/* g_grid[r] = generation r (row 0 = initial seed, row 1 = first step …) */
static uint8_t g_grid[MAX_ROWS][MAX_COLS];

static int g_rows;        /* terminal rows                               */
static int g_cols;        /* terminal cols                               */
static int g_ca_rows;     /* CA area = g_rows - 2  (title + HUD)        */
static int g_gen;         /* highest computed generation (0 = seeded)   */

static int g_rule;
static int g_cls;
static int g_preset;

static int g_delay;       /* ticks between adding one row               */
static int g_delay_ctr;
static int g_paused;
static int g_auto;
static int g_state;
static int g_pause_ctr;

static int g_idig[3];     /* digit input buffer                         */
static int g_ilen;

static const char *rule_desc(void)
{
    for (int i = 0; i < N_PRESETS; i++)
        if (PRESETS[i].rule == g_rule) return PRESETS[i].desc;
    return "user-defined";
}

static void ca_seed_center(void)
{
    memset(g_grid, 0, sizeof(g_grid));
    g_gen       = 0;
    g_state     = ST_BUILD;
    g_pause_ctr = 0;
    g_delay_ctr = 0;
    if (g_cols > 0) g_grid[0][g_cols / 2] = 1;
}

static void ca_seed_random(void)
{
    memset(g_grid, 0, sizeof(g_grid));
    g_gen       = 0;
    g_state     = ST_BUILD;
    g_pause_ctr = 0;
    g_delay_ctr = 0;
    for (int c = 0; c < g_cols; c++)
        g_grid[0][c] = (uint8_t)(rand() & 1);
}

static void ca_set_rule(int rule)
{
    g_rule   = rule;
    g_cls    = ca_classify(rule);
    g_preset = -1;
    for (int i = 0; i < N_PRESETS; i++)
        if (PRESETS[i].rule == rule) { g_preset = i; break; }
    ca_seed_center();
}

static void ca_set_preset(int idx)
{
    g_preset = ((idx % N_PRESETS) + N_PRESETS) % N_PRESETS;
    ca_set_rule(PRESETS[g_preset].rule);
}

/* Compute the next generation into g_grid[g_gen+1] and advance g_gen. */
static void ca_advance(void)
{
    if (g_gen >= g_ca_rows - 1) return;

    uint8_t *src  = g_grid[g_gen];
    uint8_t *dst  = g_grid[g_gen + 1];
    uint8_t  rule = (uint8_t)g_rule;
    int      cols = g_cols;

    for (int c = 0; c < cols; c++) {
        uint8_t l  = src[(c - 1 + cols) % cols];
        uint8_t m  = src[c];
        uint8_t rv = src[(c + 1)        % cols];
        dst[c] = (rule >> ((l << 2) | (m << 1) | rv)) & 1;
    }
    g_gen++;
}

static void sim_init(void)
{
    g_delay  = DELAY_DEF;
    g_paused = 0;
    g_auto   = 1;
    g_preset = 0;
    g_ilen   = 0;
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    ca_set_preset(0);
}

static void sim_tick(void)
{
    if (g_paused) return;

    if (g_state == ST_PAUSE) {
        g_pause_ctr++;
        if (g_pause_ctr >= PAUSE_TICKS) {
            if (g_auto) ca_set_preset(g_preset + 1);
            else        ca_seed_center();
        }
        return;
    }

    /* ST_BUILD: add one row every g_delay ticks */
    if (++g_delay_ctr >= g_delay) {
        g_delay_ctr = 0;
        ca_advance();
        if (g_gen >= g_ca_rows - 1) {
            g_state     = ST_PAUSE;
            g_pause_ctr = 0;
        }
    }
}

/* ── §5 scene ────────────────────────────────────────────────────────── */

static void scene_title(void)
{
    int cp = class_cp(g_cls);

    /* full-width reversed title bar in the class colour */
    attron(COLOR_PAIR(cp) | A_BOLD | A_REVERSE);
    mvhline(0, 0, ' ', g_cols - 1);

    /* countdown while paused between patterns */
    char hold[24] = "";
    if (g_state == ST_PAUSE) {
        int secs = (PAUSE_TICKS - g_pause_ctr + 29) / 30;
        snprintf(hold, sizeof(hold), " — next in %ds", secs);
    }

    mvprintw(0, 1, "Rule %3d: %-18s | %s | preset %d / %d%s",
             g_rule, rule_desc(), class_name(g_cls),
             g_preset + 1, N_PRESETS, hold);

    attroff(COLOR_PAIR(cp) | A_BOLD | A_REVERSE);
}

static void scene_draw(void)
{
    int cp = class_cp(g_cls);
    attron(COLOR_PAIR(cp));

    /* only draw rows that have been computed so far */
    for (int r = 0; r <= g_gen && r < g_ca_rows; r++) {
        uint8_t *row = g_grid[r];
        for (int c = 0; c < g_cols - 1; c++)
            mvaddch(1 + r, c, row[c] ? (chtype)(unsigned char)LIVE_CHAR : ' ');
    }

    attroff(COLOR_PAIR(cp));
}

static void scene_hud(void)
{
    if (g_rows < 1) return;

    /* pending digit input */
    char ibuf[8] = "";
    if (g_ilen > 0) {
        ibuf[0] = '>';
        for (int i = 0; i < g_ilen; i++) ibuf[i + 1] = (char)('0' + g_idig[i]);
        ibuf[g_ilen + 1] = '_';
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " n/p:pattern  a:%s  r:seed  R:rand  +/-:speed  "
             "spc:%s  0-9+Enter:rule#  %s  q:quit",
             g_auto ? "auto" : "man.",
             g_paused ? "resume" : "pause",
             g_ilen > 0 ? ibuf : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §6 screen ───────────────────────────────────────────────────────── */

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
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    int old_cols = g_cols;
    g_rows    = rows;
    g_cols    = cols;
    g_ca_rows = rows - 2;
    if (g_ca_rows < 1) g_ca_rows = 1;
    if (cols != old_cols) ca_seed_center();
    erase();
}

/* ── §7 app ──────────────────────────────────────────────────────────── */

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
    screen_resize();  /* sets g_cols / g_ca_rows before sim_init seeds */
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
            case 'a': case 'A': g_auto   ^= 1;  break;
            case 'n':           ca_set_preset(g_preset + 1); break;
            case 'p':           ca_set_preset(g_preset - 1); break;
            case 'r':           ca_seed_center();  break;
            case 'R':           ca_seed_random();  break;
            case '+': case '=':
                if (g_delay > DELAY_MIN) g_delay--;
                break;
            case '-': case '_':
                if (g_delay < DELAY_MAX) g_delay++;
                break;
            case KEY_BACKSPACE: case 127:
                if (g_ilen > 0) g_ilen--;
                break;
            case '\n': case '\r': case KEY_ENTER:
                if (g_ilen > 0) {
                    int val = 0;
                    for (int i = 0; i < g_ilen; i++)
                        val = val * 10 + g_idig[i];
                    if (val <= 255) ca_set_rule(val);
                    g_ilen = 0;
                }
                break;
            default:
                if (ch >= '0' && ch <= '9' && g_ilen < 3) {
                    g_idig[g_ilen++] = ch - '0';
                    if (g_ilen == 3) {  /* 3 digits → apply immediately */
                        int val = g_idig[0]*100 + g_idig[1]*10 + g_idig[2];
                        if (val <= 255) ca_set_rule(val);
                        g_ilen = 0;
                    }
                }
                break;
            }
        }

        sim_tick();

        erase();
        scene_title();
        scene_draw();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
