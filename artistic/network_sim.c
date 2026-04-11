/*
 * network_sim.c — SIR Epidemic on a Watts-Strogatz Small-World Network
 *
 * N=80 nodes are arranged in a ring.  A Watts-Strogatz random graph is
 * constructed: each node is first connected to its K=4 nearest ring
 * neighbours, then each edge is independently rewired to a random node
 * with probability p=0.1.  The result is a "small-world" network with
 * short average path length and high clustering.
 *
 * SIR compartmental model (discrete-time stochastic):
 *   S (susceptible) — white 'o'   : can be infected
 *   I (infected)    — red   'O'   : infects neighbours with prob β per tick
 *   R (recovered)   — green 'o'   : immune; removed from dynamics
 *
 * Basic reproduction number R0 = β · <k> / γ  where <k> is mean degree.
 *
 * Keys:  q quit   ↑↓ β (infection prob)   ←→ γ (recovery prob)
 *        r reset epidemic   i inject new infected node   spc pause
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/network_sim.c \
 *       -o network_sim -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 graph  §5 SIR  §6 draw  §7 app
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_NODES   80      /* ring nodes                                    */
#define WS_K       4      /* initial ring degree (2 on each side)          */
#define WS_P       0.10f  /* Watts-Strogatz rewiring probability           */

#define BETA_INIT  0.030f  /* infection probability per S-I edge per tick  */
#define GAMMA_INIT 0.020f  /* recovery probability per I node per tick     */
#define BETA_STEP  0.005f
#define GAMMA_STEP 0.005f

#define CELL_W      8
#define CELL_H     16
#define HUD_ROWS    3
#define RING_FRAC   0.42f  /* ring radius as fraction of min(pw/2, ph/2)   */
#define RENDER_NS  (1000000000LL / 15)   /* 15 fps — slower to see SIR     */

typedef enum { S_STATE, I_STATE, R_STATE } SIR;

enum {
    CP_S = 1, CP_I, CP_R,
    CP_EDGE, CP_REWIRE,
    CP_HUD, CP_BAR_S, CP_BAR_I, CP_BAR_R,
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_S,      250, -1);   /* light grey — susceptible          */
        init_pair(CP_I,      196, -1);   /* bright red — infected             */
        init_pair(CP_R,       46, -1);   /* bright green — recovered          */
        init_pair(CP_EDGE,   238, -1);   /* dark grey — regular ring edge      */
        init_pair(CP_REWIRE, 240, -1);   /* mid grey — rewired long edge       */
        init_pair(CP_HUD,    244, -1);   /* grey — HUD                        */
        init_pair(CP_BAR_S,  250, -1);
        init_pair(CP_BAR_I,  196, -1);
        init_pair(CP_BAR_R,   46, -1);
    } else {
        init_pair(CP_S,      COLOR_WHITE,  -1);
        init_pair(CP_I,      COLOR_RED,    -1);
        init_pair(CP_R,      COLOR_GREEN,  -1);
        init_pair(CP_EDGE,   COLOR_WHITE,  -1);
        init_pair(CP_REWIRE, COLOR_WHITE,  -1);
        init_pair(CP_HUD,    COLOR_WHITE,  -1);
        init_pair(CP_BAR_S,  COLOR_WHITE,  -1);
        init_pair(CP_BAR_I,  COLOR_RED,    -1);
        init_pair(CP_BAR_R,  COLOR_GREEN,  -1);
    }
}

/* ===================================================================== */
/* §4  graph — Watts-Strogatz construction                                */
/* ===================================================================== */

static bool  g_adj[N_NODES][N_NODES];
static bool  g_is_rewired[N_NODES][N_NODES];   /* true = long-range edge */
static float g_node_px[N_NODES], g_node_py[N_NODES];
static int   g_rows, g_cols;

static void graph_build(void)
{
    memset(g_adj, 0, sizeof(g_adj));
    memset(g_is_rewired, 0, sizeof(g_is_rewired));

    /* Step 1: regular ring lattice, K/2 neighbours per side */
    for (int i = 0; i < N_NODES; i++) {
        for (int k = 1; k <= WS_K / 2; k++) {
            int j = (i + k) % N_NODES;
            g_adj[i][j] = g_adj[j][i] = true;
        }
    }

    /* Step 2: Watts-Strogatz rewiring */
    for (int i = 0; i < N_NODES; i++) {
        for (int k = 1; k <= WS_K / 2; k++) {
            if ((float)rand() / (float)RAND_MAX >= WS_P) continue;
            int old_j = (i + k) % N_NODES;
            /* choose a new random target that is not i and not already connected */
            int new_j;
            int tries = 0;
            do {
                new_j = rand() % N_NODES;
                tries++;
            } while ((new_j == i || g_adj[i][new_j]) && tries < N_NODES);
            if (tries >= N_NODES) continue;
            /* remove old edge, add new edge */
            g_adj[i][old_j] = g_adj[old_j][i] = false;
            g_adj[i][new_j] = g_adj[new_j][i] = true;
            g_is_rewired[i][new_j] = g_is_rewired[new_j][i] = true;
        }
    }
}

/* compute mean degree for R0 */
static float mean_degree(void)
{
    int total = 0;
    for (int i = 0; i < N_NODES; i++)
        for (int j = 0; j < N_NODES; j++)
            if (g_adj[i][j]) total++;
    return (float)total / (float)N_NODES;
}

static void layout_ring(int rows, int cols)
{
    int pw = cols * CELL_W, ph = rows * CELL_H;
    float cx = (float)pw * 0.5f;
    float cy = (float)(ph + HUD_ROWS * CELL_H) * 0.5f;
    float r  = RING_FRAC * (float)(pw < ph ? pw : ph) * 0.5f;

    for (int i = 0; i < N_NODES; i++) {
        float a = 2.f * (float)M_PI * (float)i / (float)N_NODES - (float)M_PI/2.f;
        g_node_px[i] = cx + r * cosf(a);
        g_node_py[i] = cy + r * sinf(a);
    }
}

static int px_cx(float px) { return (int)(px / (float)CELL_W + 0.5f); }
static int px_cy(float py) { return (int)(py / (float)CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  SIR dynamics                                                       */
/* ===================================================================== */

static SIR   g_state[N_NODES];
static float g_beta  = BETA_INIT;
static float g_gamma = GAMMA_INIT;
static bool  g_paused = false;
static int   g_tick   = 0;

static int count_s(void){ int n=0; for(int i=0;i<N_NODES;i++) n+=(g_state[i]==S_STATE); return n; }
static int count_i(void){ int n=0; for(int i=0;i<N_NODES;i++) n+=(g_state[i]==I_STATE); return n; }
static int count_r(void){ int n=0; for(int i=0;i<N_NODES;i++) n+=(g_state[i]==R_STATE); return n; }

static void sir_reset(void)
{
    for (int i = 0; i < N_NODES; i++) g_state[i] = S_STATE;
    /* seed one infected node at random */
    g_state[rand() % N_NODES] = I_STATE;
    g_tick = 0;
}

static void sir_inject(void)
{
    /* inject a new infected node into a random susceptible */
    int tries = 0;
    while (tries++ < N_NODES * 2) {
        int i = rand() % N_NODES;
        if (g_state[i] == S_STATE) { g_state[i] = I_STATE; return; }
    }
}

static void sir_tick(void)
{
    if (g_paused) return;
    SIR new_state[N_NODES];
    memcpy(new_state, g_state, sizeof(g_state));

    for (int i = 0; i < N_NODES; i++) {
        if (g_state[i] == I_STATE) {
            /* recovery */
            if ((float)rand() / (float)RAND_MAX < g_gamma)
                new_state[i] = R_STATE;
            /* infect susceptible neighbours */
            for (int j = 0; j < N_NODES; j++) {
                if (!g_adj[i][j]) continue;
                if (g_state[j] != S_STATE) continue;
                if ((float)rand() / (float)RAND_MAX < g_beta)
                    new_state[j] = I_STATE;
            }
        }
    }
    memcpy(g_state, new_state, sizeof(g_state));
    g_tick++;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

/*
 * draw_line() — Bresenham line, directional character selection.
 */
static void draw_line(int x0, int y0, int x1, int y1,
                      attr_t attr, int cols, int rows)
{
    int dx=abs(x1-x0), dy=abs(y1-y0);
    int sx=x0<x1?1:-1, sy=y0<y1?1:-1, err=dx-dy;
    for(;;){
        if(x0>=0&&x0<cols&&y0>=0&&y0<rows){
            int e2=2*err; bool bx=e2>-dy, by=e2<dx;
            chtype ch=(bx&&by)?(sx==sy?'\\':'/'):(bx?'-':'|');
            attron(attr); mvaddch(y0,x0,ch); attroff(attr);
        }
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>-dy){err-=dy;x0+=sx;}
        if(e2< dx){err+=dx;y0+=sy;}
    }
}

static void scene_draw(int rows, int cols)
{
    /* ── edges ───────────────────────────────────────────────────── */
    for (int i = 0; i < N_NODES; i++) {
        for (int j = i+1; j < N_NODES; j++) {
            if (!g_adj[i][j]) continue;
            /* colour edge by infection: I-I or I-S edge → red tint */
            bool hot = (g_state[i]==I_STATE || g_state[j]==I_STATE);
            int  cp  = hot ? CP_I :
                       g_is_rewired[i][j] ? CP_REWIRE : CP_EDGE;
            draw_line(px_cx(g_node_px[i]), px_cy(g_node_py[i]),
                      px_cx(g_node_px[j]), px_cy(g_node_py[j]),
                      COLOR_PAIR(cp), cols, rows);
        }
    }

    /* ── nodes ───────────────────────────────────────────────────── */
    for (int i = 0; i < N_NODES; i++) {
        int cx = px_cx(g_node_px[i]);
        int cy = px_cy(g_node_py[i]);
        if (cx<0||cx>=cols||cy<0||cy>=rows) continue;
        int   cp;
        chtype ch;
        attr_t ex = 0;
        switch (g_state[i]) {
        case S_STATE: cp=CP_S; ch='o'; break;
        case I_STATE: cp=CP_I; ch='O'; ex=A_BOLD; break;
        case R_STATE: cp=CP_R; ch='o'; break;
        default:      cp=CP_S; ch='o'; break;
        }
        attron(COLOR_PAIR(cp)|ex);
        mvaddch(cy, cx, ch);
        attroff(COLOR_PAIR(cp)|ex);
    }

    /* ── HUD ─────────────────────────────────────────────────────── */
    int ns = count_s(), ni = count_i(), nr = count_r();
    float mk = mean_degree();
    float r0 = (g_gamma > 0.f) ? g_beta * mk / g_gamma : 0.f;

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " NetworkSIM  q:quit  ↑↓:β  ←→:γ  r:reset  i:inject  spc:pause");
    mvprintw(1, 0,
        " β=%.3f  γ=%.3f  R0=%.2f  <k>=%.1f  tick=%d  nodes=%d  %s",
        g_beta, g_gamma, r0, mk, g_tick, N_NODES,
        g_paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(CP_HUD));

    /* SIR count bar */
    {
        int bar_w = cols - 20;
        if (bar_w < 10) bar_w = 10;
        int fill_s = ns * bar_w / N_NODES;
        int fill_i = ni * bar_w / N_NODES;
        int fill_r = nr * bar_w / N_NODES;

        mvprintw(2, 0, " S:%-3d ", ns);
        int bx = 6;
        attron(COLOR_PAIR(CP_BAR_S));
        for (int i=0;i<fill_s&&bx+i<cols;i++) mvaddch(2, bx+i, '=');
        attroff(COLOR_PAIR(CP_BAR_S));
        bx += fill_s;

        attron(COLOR_PAIR(CP_BAR_I)|A_BOLD);
        for (int i=0;i<fill_i&&bx+i<cols;i++) mvaddch(2, bx+i, '#');
        attroff(COLOR_PAIR(CP_BAR_I)|A_BOLD);
        bx += fill_i;

        attron(COLOR_PAIR(CP_BAR_R));
        for (int i=0;i<fill_r&&bx+i<cols;i++) mvaddch(2, bx+i, '-');
        attroff(COLOR_PAIR(CP_BAR_R));
        bx += fill_r;

        attron(COLOR_PAIR(CP_HUD));
        mvprintw(2, bx + 2, " I:%-3d R:%-3d", ni, nr);
        attroff(COLOR_PAIR(CP_HUD));
    }
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s==SIGINT||s==SIGTERM) g_quit=1;
    if (s==SIGWINCH)           g_resize=1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)(clock_ns() & 0xFFFFFFFF));

    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    graph_build();
    layout_ring(rows, cols);
    sir_reset();

    long long last = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            g_rows = rows; g_cols = cols;
            layout_ring(rows, cols);
            last = clock_ns();
            continue;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1;  break;
        case ' ':  g_paused = !g_paused;           break;
        case 'r': case 'R':  sir_reset();          break;
        case 'i': case 'I':  sir_inject();         break;
        case KEY_UP:
            g_beta += BETA_STEP;
            if (g_beta > 1.f) g_beta = 1.f;
            break;
        case KEY_DOWN:
            g_beta -= BETA_STEP;
            if (g_beta < 0.f) g_beta = 0.f;
            break;
        case KEY_RIGHT:
            g_gamma += GAMMA_STEP;
            if (g_gamma > 1.f) g_gamma = 1.f;
            break;
        case KEY_LEFT:
            g_gamma -= GAMMA_STEP;
            if (g_gamma < 0.f) g_gamma = 0.f;
            break;
        default: break;
        }

        long long now = clock_ns();
        long long dt  = now - last;
        last = now;
        (void)dt;

        sir_tick();

        erase();
        scene_draw(rows, cols);
        wnoutrefresh(stdscr);
        doupdate();

        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }

    return 0;
}
