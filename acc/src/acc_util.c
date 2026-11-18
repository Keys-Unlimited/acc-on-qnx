/* acc_util.c - helpers shared by all ACC processes */
#include "acc_common.h"
#include <sys/syspage.h>

static stats_shm_t   *g_stats;
static pthread_once_t g_once = PTHREAD_ONCE_INIT;
static double         g_cps  = 1e9;

static void init_once(void)
{
    g_cps   = (double)SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    g_stats = shm_map(SHM_STATS, sizeof(stats_shm_t));
    if (g_stats) g_stats->nslots = NUM_SLOTS;
}

stats_shm_t *stats_attach(void)
{
    pthread_once(&g_once, init_once);
    return g_stats;
}

uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

uint64_t cyc_now(void) { return ClockCycles(); }

double cyc_to_ns(uint64_t cycles)
{
    pthread_once(&g_once, init_once);
    return (double)cycles * 1e9 / g_cps;
}

void *shm_map(const char *name, size_t size)
{
    int fd = shm_open(name, O_RDWR | O_CREAT, 0666);
    if (fd < 0) { perror("shm_open"); return NULL; }
    if (ftruncate(fd, (off_t)size) < 0) { perror("ftruncate"); close(fd); return NULL; }
    void *p = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) { perror("mmap"); return NULL; }
    return p;
}

int name_open_retry(const char *name, int tries)
{
    for (int i = 0; i < tries; i++) {
        int c = name_open(name, 0);
        if (c >= 0) return c;
        usleep(200000);
    }
    return -1;
}

int lazy_open(int *coid, const char *name)
{
    if (*coid >= 0) return *coid;
    for (int i = 0; i < 10; i++) {
        int c = name_open(name, 0);
        if (c >= 0) { *coid = c; return c; }
        usleep(100000);
    }
    return -1;
}

int rt_set_self(int prio)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    int pol = (prio >= 35) ? SCHED_FIFO : SCHED_RR;
    int rc = pthread_setschedparam(pthread_self(), pol, &sp);
    if (rc != 0)
        fprintf(stderr, "warning: could not set priority %d (rc=%d); run as root\n", prio, rc);
    return rc;
}

int rt_spawn(pthread_t *t, void *(*fn)(void *), void *arg)
{
    return pthread_create(t, NULL, fn, arg);
}

const char *mode_name(int m)
{
    switch (m) {
    case MODE_OFF:       return "OFF";
    case MODE_SPEED:     return "SPEED";
    case MODE_FOLLOW:    return "FOLLOW";
    case MODE_EMERGENCY: return "EMERGENCY";
    case MODE_FAULT:     return "FAULT";
    default:             return "?";
    }
}

/* ---------------- periodic timing with jitter / deadline stats ---------------- */

int periodic_setup(periodic_t *p, int chid, unsigned period_ms, int slot, const char *name)
{
    memset(p, 0, sizeof *p);
    p->period_ns = (uint64_t)period_ms * 1000000ULL;
    p->chid = chid ? chid : ChannelCreate(0);
    if (p->chid < 0) { perror("ChannelCreate"); return -1; }
    p->coid = ConnectAttach(0, 0, p->chid, _NTO_SIDE_CHANNEL, 0);
    if (p->coid < 0) { perror("ConnectAttach"); return -1; }

    stats_shm_t *s = stats_attach();
    if (s && slot >= 0 && slot < NUM_SLOTS) {
        p->st = &s->slot[slot];
        strncpy(p->st->name, name, sizeof p->st->name - 1);
    }

    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, p->coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_TICK, 0);
    if (timer_create(CLOCK_MONOTONIC, &ev, &p->tid) < 0) { perror("timer_create"); return -1; }

    struct itimerspec it;
    it.it_value.tv_sec     = (time_t)(p->period_ns / 1000000000ULL);
    it.it_value.tv_nsec    = (long)(p->period_ns % 1000000000ULL);
    it.it_interval         = it.it_value;
    p->t0_ns = now_ns();
    if (timer_settime(p->tid, 0, &it, NULL) < 0) { perror("timer_settime"); return -1; }
    return 0;
}

void periodic_on_tick(periodic_t *p)
{
    uint64_t now = now_ns();
    p->k++;
    p->exp_ns = p->t0_ns + p->k * p->period_ns;
    int64_t jit = (int64_t)(now - p->exp_ns);
    if (jit < 0) jit = -jit;
    if (p->st) {
        stat_slot_t *s = p->st;
        __atomic_store_n(&s->ticks, p->k, __ATOMIC_RELAXED);
        if (jit > s->max_jitter_ns) __atomic_store_n(&s->max_jitter_ns, jit, __ATOMIC_RELAXED);
        __atomic_store_n(&s->sum_jitter_ns, s->sum_jitter_ns + (uint64_t)jit, __ATOMIC_RELAXED);
    }
    p->cyc_wake = cyc_now();
    p->started  = 1;
}

void periodic_done(periodic_t *p)
{
    if (!p->started) return;
    p->started = 0;
    int64_t exec = (int64_t)cyc_to_ns(cyc_now() - p->cyc_wake);
    uint64_t end = now_ns();
    if (p->st) {
        stat_slot_t *s = p->st;
        if (exec > s->max_exec_ns) __atomic_store_n(&s->max_exec_ns, exec, __ATOMIC_RELAXED);
        __atomic_store_n(&s->sum_exec_ns, s->sum_exec_ns + (uint64_t)exec, __ATOMIC_RELAXED);
        if (end > p->exp_ns + p->period_ns)
            __atomic_store_n(&s->misses, s->misses + 1, __ATOMIC_RELAXED);
    }
}

void periodic_wait(periodic_t *p)
{
    struct _pulse pulse;
    periodic_done(p);
    for (;;) {
        int rcvid = MsgReceive(p->chid, &pulse, sizeof pulse, NULL);
        if (rcvid == 0 && pulse.code == PULSE_TICK) break;
        if (rcvid < 0 && errno != EINTR) { perror("periodic MsgReceive"); sleep(1); }
    }
    periodic_on_tick(p);
}
