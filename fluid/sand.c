/*
 * sand.c  —  ncurses falling sand simulation
 *
 * Sand pours from a source at the top, falls under gravity,
 * and piles into natural slopes.
 *
 * Enhancements:
 *
 *   Wind — horizontal force that drifts falling grains sideways.
 *     w / W   wind right / left
 *     0       calm
 *     Wind deflects the emitter spray so the stream leans.
 *
 *   Per-grain age — each grain accumulates age each tick it is
 *     stationary.  Age drives the visual character:
 *       newly falling / spawned   '`'  pale yellow   (airborne)
 *       just landed               '.'  pale yellow   (fresh)
 *       settling                  'o'  golden        (light pack)
 *       settled surface           'O'  amber         (mid pack)
 *       compressed mid            '0'  dark amber    (packed)
 *       deep compressed base      '#'  dark brown    (dense)
 *     New grains are bright and individual.  Buried grains darken
 *     and show as dense packed material.
 *
 * Keys:
 *   q / ESC     quit
 *   space       toggle emitter on / off
 *   p           pause / resume
 *   r           clear all sand
 *   Left/Right  move the emitter
 *   =  -        wider / narrower emitter
 *   ]  [        simulation faster / slower
 *   w           wind right (press multiple times to strengthen)
 *   W           wind left
 *   0           calm — no wind
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra sand.c -o sand -lncurses
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid   — CA + per-grain age + wind drift
 *   §5  source
 *   §6  scene
 *   §7  screen — single stdscr
 *   §8  app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 30,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    HUD_COLS         = 54,
    FPS_UPDATE_MS    = 500,

    SOURCE_ROW       =  1,
    SOURCE_W_DEFAULT =  3,
    SOURCE_W_MIN     =  1,
    SOURCE_W_MAX     = 30,

    WIND_MAX         =  3,   /* max wind strength                        */

    /* Age thresholds — stationary ticks before next visual level */
    AGE_DOT    =   3,
    AGE_SMALL  =  12,
    AGE_MID    =  30,
    AGE_PACK   =  60,
    AGE_DENSE  = 120,
    AGE_MAX    = 200,
};

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(f)    (NS_PER_SEC/(f))

/* Wind drift probability denominator: P(drift) = abs(wind) / WIND_PROB_DEN */
#define WIND_PROB_DEN  4

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

enum {
    CP_NEW    = 1,   /* very pale yellow — airborne / spawned           */
    CP_GRAIN  = 2,   /* pale yellow      — freshly landed               */
    CP_LIGHT  = 3,   /* golden           — settling                     */
    CP_MID    = 4,   /* amber            — settled                      */
    CP_PACK   = 5,   /* dark amber       — packed mid-layer             */
    CP_DENSE  = 6,   /* dark brown       — compressed base              */
    CP_SOURCE = 7,   /* bright yellow    — emitter marker               */
    CP_WIND   = 8,   /* light blue       — wind indicator               */
};

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(CP_NEW,    230, COLOR_BLACK);
        init_pair(CP_GRAIN,  229, COLOR_BLACK);
        init_pair(CP_LIGHT,  220, COLOR_BLACK);
        init_pair(CP_MID,    178, COLOR_BLACK);
        init_pair(CP_PACK,   136, COLOR_BLACK);
        init_pair(CP_DENSE,  130, COLOR_BLACK);
        init_pair(CP_SOURCE, 226, COLOR_BLACK);
        init_pair(CP_WIND,   117, COLOR_BLACK);
    } else {
        init_pair(CP_NEW,    COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_GRAIN,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_LIGHT,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_MID,    COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_PACK,   COLOR_RED,    COLOR_BLACK);
        init_pair(CP_DENSE,  COLOR_RED,    COLOR_BLACK);
        init_pair(CP_SOURCE, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_WIND,   COLOR_CYAN,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid  — CA + per-grain age + wind                                 */
/* ===================================================================== */

/*
 * Each cell is 0=empty or 1=sand.
 * A parallel age[] array tracks how many consecutive ticks each grain
 * has been stationary.  Age drives the visual character selection.
 * Age resets to 0 when a grain moves; increments by 1 when at rest.
 */

typedef uint8_t Cell;

typedef struct {
    Cell    *cur;      /* [rows*cols]  0=empty  1=sand                  */
    Cell    *nxt;      /* [rows*cols]  next state                        */
    uint8_t *age;      /* [rows*cols]  stationary ticks                  */
    uint8_t *nxt_age;  /* [rows*cols]  age for next state                */
    bool    *moved;    /* [rows*cols]  processed this tick?              */
    int      cols;
    int      rows;
    int      wind;     /* -WIND_MAX..+WIND_MAX  positive=right           */
} Grid;

static void grid_alloc(Grid *g, int cols, int rows)
{
    g->cols    = cols;
    g->rows    = rows;
    g->cur     = calloc((size_t)(cols*rows), sizeof(Cell));
    g->nxt     = calloc((size_t)(cols*rows), sizeof(Cell));
    g->age     = calloc((size_t)(cols*rows), sizeof(uint8_t));
    g->nxt_age = calloc((size_t)(cols*rows), sizeof(uint8_t));
    g->moved   = calloc((size_t)(cols*rows), sizeof(bool));
    g->wind    = 0;
}
static void grid_free(Grid *g)
{
    free(g->cur); free(g->nxt);
    free(g->age); free(g->nxt_age);
    free(g->moved);
    *g = (Grid){0};
}
static void grid_clear(Grid *g)
{
    size_t n = (size_t)(g->cols * g->rows);
    memset(g->cur,     0, n*sizeof(Cell));
    memset(g->nxt,     0, n*sizeof(Cell));
    memset(g->age,     0, n*sizeof(uint8_t));
    memset(g->nxt_age, 0, n*sizeof(uint8_t));
    memset(g->moved,   0, n*sizeof(bool));
}

static inline bool gin(const Grid *g, int x, int y)
    { return x>=0 && x<g->cols && y>=0 && y<g->rows; }
static inline int  gidx(const Grid *g, int x, int y)
    { return y*g->cols+x; }
static inline Cell gget(const Grid *g, int x, int y)
    { if (!gin(g,x,y)) return 1; return g->cur[gidx(g,x,y)]; }
static inline void gset_cur(Grid *g, int x, int y, Cell v)
    { if (gin(g,x,y)) g->cur[gidx(g,x,y)]=v; }
static inline bool gmoved(const Grid *g, int x, int y)
    { return gin(g,x,y) && g->moved[gidx(g,x,y)]; }
static inline void gmark(Grid *g, int x, int y)
    { if (gin(g,x,y)) g->moved[gidx(g,x,y)]=true; }

/* Move grain (sx,sy)→(dx,dy): clear source, set dest, reset age, mark both */
static void gmove(Grid *g, int sx, int sy, int dx, int dy)
{
    g->nxt[gidx(g,sx,sy)]     = 0;
    g->nxt_age[gidx(g,sx,sy)] = 0;
    g->nxt[gidx(g,dx,dy)]     = 1;
    g->nxt_age[gidx(g,dx,dy)] = 0;
    gmark(g,sx,sy); gmark(g,dx,dy);
}

static void grid_update_cell(Grid *g, int x, int y)
{
    if (gget(g,x,y) != 1) return;
    if (gmoved(g,x,y))    return;

    int i = gidx(g,x,y);

    /* 1. Straight down */
    if (gin(g,x,y+1) && gget(g,x,y+1)==0 && !gmoved(g,x,y+1)) {
        gmove(g,x,y,x,y+1); return;
    }

    /* 2. Diagonal down — random priority */
    int a = (rand()&1) ? -1 : 1, b = -a;
    if (gin(g,x+a,y+1) && gget(g,x+a,y+1)==0 && !gmoved(g,x+a,y+1)) {
        gmove(g,x,y,x+a,y+1); return;
    }
    if (gin(g,x+b,y+1) && gget(g,x+b,y+1)==0 && !gmoved(g,x+b,y+1)) {
        gmove(g,x,y,x+b,y+1); return;
    }

    /* 3. Wind drift — only light/young grains (not yet settled) */
    if (g->wind != 0 && g->age[i] < AGE_SMALL) {
        int dir  = (g->wind > 0) ? 1 : -1;
        int wabs = abs(g->wind);
        if ((rand() % WIND_PROB_DEN) < wabs) {
            int wx = x + dir;
            if (gin(g,wx,y) && gget(g,wx,y)==0 && !gmoved(g,wx,y)) {
                gmove(g,x,y,wx,y); return;
            }
        }
    }

    /* 4. At rest — copy, increment age */
    g->nxt[i]     = 1;
    g->nxt_age[i] = (g->age[i] < AGE_MAX) ? g->age[i]+1 : AGE_MAX;
    gmark(g,x,y);
}

static void grid_tick(Grid *g)
{
    int    cols = g->cols, rows = g->rows;
    size_t n    = (size_t)(cols*rows);

    memset(g->nxt,     0, n*sizeof(Cell));
    memset(g->nxt_age, 0, n*sizeof(uint8_t));
    memset(g->moved,   0, n*sizeof(bool));

    int *order = malloc((size_t)cols * sizeof(int));
    for (int x = 0; x < cols; x++) order[x] = x;

    for (int y = rows-1; y >= 0; y--) {
        for (int i = cols-1; i > 0; i--) {
            int j = rand()%(i+1);
            int t = order[i]; order[i]=order[j]; order[j]=t;
        }
        for (int i = 0; i < cols; i++)
            grid_update_cell(g, order[i], y);
    }
    free(order);

    Cell    *tc = g->cur;     g->cur     = g->nxt;     g->nxt     = tc;
    uint8_t *ta = g->age;     g->age     = g->nxt_age; g->nxt_age = ta;
}

static int grid_neighbors(const Grid *g, int x, int y)
{
    int n = 0;
    for (int dy=-1; dy<=1; dy++)
        for (int dx=-1; dx<=1; dx++)
            if ((dx||dy) && gin(g,x+dx,y+dy))
                n += g->cur[gidx(g,x+dx,y+dy)];
    return n;
}

/*
 * grain_visual() — map (age, neighbour_count) → (char, attr).
 *
 * Age drives the primary level selection.  Neighbour count suppresses
 * the lightest chars for grains surrounded by settled sand — prevents
 * a freshly dropped grain inside a pile from glowing bright.
 */
static void grain_visual(uint8_t age, int nb, char *ch_out, attr_t *attr_out)
{
    int eff = (int)age;
    if (nb >= 5 && eff < AGE_MID)   eff = AGE_MID;
    if (nb >= 3 && eff < AGE_SMALL) eff = AGE_SMALL;

    char   ch;
    attr_t attr;
    if      (eff < AGE_DOT)   { ch='`'; attr=COLOR_PAIR(CP_NEW)  |A_BOLD; }
    else if (eff < AGE_SMALL) { ch='.'; attr=COLOR_PAIR(CP_GRAIN)|A_BOLD; }
    else if (eff < AGE_MID)   { ch='o'; attr=COLOR_PAIR(CP_LIGHT)|A_BOLD; }
    else if (eff < AGE_PACK)  { ch='O'; attr=COLOR_PAIR(CP_MID);          }
    else if (eff < AGE_DENSE) { ch='0'; attr=COLOR_PAIR(CP_PACK);         }
    else                      { ch='#'; attr=COLOR_PAIR(CP_DENSE);        }
    *ch_out   = ch;
    *attr_out = attr;
}

/* ===================================================================== */
/* §5  source                                                             */
/* ===================================================================== */

typedef struct { int col, w; bool on; } Source;

static void source_init(Source *s, int cols)
{
    s->col = cols/2;
    s->w   = SOURCE_W_DEFAULT;
    s->on  = true;
}

static void source_emit(const Source *s, Grid *g)
{
    if (!s->on) return;
    int half        = s->w / 2;
    int wind_offset = g->wind;   /* lean stream into wind */
    for (int dx = -half; dx <= half; dx++) {
        int x = s->col + dx + wind_offset;
        if (x < 0) x = 0;
        if (x >= g->cols) x = g->cols-1;
        if (gin(g,x,SOURCE_ROW) && gget(g,x,SOURCE_ROW)==0) {
            gset_cur(g,x,SOURCE_ROW,1);
            g->age[gidx(g,x,SOURCE_ROW)] = 0;
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Grid grid; Source source; bool paused; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s,0,sizeof*s);
    grid_alloc(&s->grid, cols, rows);
    source_init(&s->source, cols);
}
static void scene_free(Scene *s)  { grid_free(&s->grid); }
static void scene_resize(Scene *s, int cols, int rows)
{
    int wind = s->grid.wind;
    grid_free(&s->grid);
    grid_alloc(&s->grid, cols, rows);
    s->grid.wind = wind;
    if (s->source.col >= cols) s->source.col = cols/2;
}
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    source_emit(&s->source, &s->grid);
    grid_tick(&s->grid);
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    const Grid *g    = &s->grid;
    int         cols = g->cols;
    int         rows = g->rows;

    /* Sand grains */
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (g->cur[gidx(g,x,y)] == 0) continue;
            uint8_t age = g->age[gidx(g,x,y)];
            int     nb  = grid_neighbors(g,x,y);
            char   ch; attr_t attr;
            grain_visual(age, nb, &ch, &attr);
            wattron(w,attr);
            mvwaddch(w,y,x,(chtype)(unsigned char)ch);
            wattroff(w,attr);
        }
    }

    /* Emitter marker — lean with wind */
    if (s->source.on) {
        int half = s->source.w/2;
        int wo   = g->wind;
        wattron(w, COLOR_PAIR(CP_SOURCE)|A_BOLD);
        for (int dx = -half; dx <= half; dx++) {
            int mx = s->source.col + dx + wo;
            if (mx>=0 && mx<cols)
                mvwaddch(w, SOURCE_ROW-1, mx, 'v');
        }
        wattroff(w, COLOR_PAIR(CP_SOURCE)|A_BOLD);
    }

    /* Wind indicator — arrows at bottom corners */
    if (g->wind != 0) {
        char wc   = (g->wind > 0) ? '>' : '<';
        int  wabs = abs(g->wind);
        wattron(w, COLOR_PAIR(CP_WIND)|A_BOLD);
        for (int i = 0; i < wabs && i < cols/4; i++) {
            int wx = (g->wind > 0) ? cols-1-i : i;
            if (wx>=0 && wx<cols && rows>1)
                mvwaddch(w, rows-1, wx, (chtype)(unsigned char)wc);
        }
        wattroff(w, COLOR_PAIR(CP_WIND)|A_BOLD);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr,TRUE); keypad(stdscr,TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr,s->rows,s->cols); }

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    char buf[HUD_COLS+1];
    const char *wstr = sc->grid.wind > 0 ? ">>>" :
                       sc->grid.wind < 0 ? "<<<" : "---";
    snprintf(buf, sizeof buf,
             "%5.1f fps  %s  emit:%s  w:%d  wind:%s  sim:%d",
             fps,
             sc->paused ? "PAUSED" : "run",
             sc->source.on ? "on" : "off",
             sc->source.w, wstr, sim_fps);
    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_LIGHT)|A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_LIGHT)|A_BOLD);
    attron(COLOR_PAIR(CP_GRAIN));
    mvprintw(1, hx, "w/W=wind  0=calm  space=emit  r=clear  p=pause");
    attroff(COLOR_PAIR(CP_GRAIN));
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit_signal(int s)   { (void)s; g_app.running=0;     }
static void on_resize_signal(int s) { (void)s; g_app.need_resize=1; }
static void cleanup(void)           { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene  *sc  = &app->scene;
    Source *src = &sc->source;
    Grid   *g   = &sc->grid;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':   src->on    = !src->on;       break;
    case 'p': case 'P': sc->paused = !sc->paused; break;
    case 'r': case 'R': grid_clear(g);       break;

    case KEY_LEFT:
        src->col--; if (src->col<1) src->col=1; break;
    case KEY_RIGHT:
        src->col++; if (src->col>=g->cols-1) src->col=g->cols-2; break;

    case '=': case '+':
        src->w++; if (src->w>SOURCE_W_MAX) src->w=SOURCE_W_MAX; break;
    case '-':
        src->w--; if (src->w<SOURCE_W_MIN) src->w=SOURCE_W_MIN; break;

    case 'w':
        g->wind++; if (g->wind> WIND_MAX) g->wind= WIND_MAX; break;
    case 'W':
        g->wind--; if (g->wind<-WIND_MAX) g->wind=-WIND_MAX; break;
    case '0':
        g->wind = 0; break;

    case ']':
        app->sim_fps+=SIM_FPS_STEP;
        if (app->sim_fps>SIM_FPS_MAX) app->sim_fps=SIM_FPS_MAX; break;
    case '[':
        app->sim_fps-=SIM_FPS_STEP;
        if (app->sim_fps<SIM_FPS_MIN) app->sim_fps=SIM_FPS_MIN; break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100*NS_PER_MS) dt = 100*NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS*NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum/(double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;

        int64_t elapsed = clock_ns()-frame_time+dt;
        clock_sleep_ns(NS_PER_SEC/60 - elapsed);
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
