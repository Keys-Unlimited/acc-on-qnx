/*
 * diag_proc - diagnostics (1 thread, lowest priority)
 *
 * Every 100 ms it samples the per-thread timing statistics that all other
 * processes keep in shared memory (single writer per slot, no locks) and
 * appends them to <logdir>/diag.csv. On SIGTERM/SIGINT it writes
 * <logdir>/diag_summary.txt and prints it.
 *
 * usage: diag_proc [logdir]     (default ./logs)
 */
#include "acc_common.h"

static volatile sig_atomic_t g_run = 1;
static void on_sig(int s) { (void)s; g_run = 0; }

static void snapshot(FILE *csv, stats_shm_t *s, double t_s)
{
    for (int i = 0; i < NUM_SLOTS; i++) {
        stat_slot_t c = s->slot[i];
        if (c.name[0] == 0 || c.ticks == 0) continue;
        fprintf(csv, "%.1f,%s,%llu,%llu,%.1f,%.1f,%.1f,%.1f\n", t_s, c.name,
                (unsigned long long)c.ticks, (unsigned long long)c.misses,
                c.max_jitter_ns / 1e3, (double)c.sum_jitter_ns / (double)c.ticks / 1e3,
                c.max_exec_ns / 1e3, (double)c.sum_exec_ns / (double)c.ticks / 1e3);
    }
}

int main(int argc, char **argv)
{
    const char *dir = (argc > 1) ? argv[1] : "./logs";
    char path[256];
    snprintf(path, sizeof path, "%s/diag.csv", dir);
    FILE *csv = fopen(path, "w");
    if (!csv) { perror("diag.csv"); return 1; }
    fprintf(csv, "t_s,thread,ticks,deadline_misses,max_jitter_us,mean_jitter_us,max_exec_us,mean_exec_us\n");

    stats_shm_t *s = stats_attach();
    if (!s) return 1;

    signal(SIGTERM, on_sig);
    signal(SIGINT, on_sig);

    rt_set_self(PRIO_DIAG);                                   /* main thread = diag_thread */
    periodic_t p;
    periodic_setup(&p, 0, DIAG_PERIOD_MS, SL_DIAG, "diag_thread");

    uint64_t t0 = now_ns();
    while (g_run) {
        periodic_wait(&p);
        snapshot(csv, s, (now_ns() - t0) / 1e9);
        fflush(csv);
    }

    snprintf(path, sizeof path, "%s/diag_summary.txt", dir);
    FILE *sum = fopen(path, "w");
    FILE *outs[2] = { stdout, sum };
    for (int k = 0; k < 2; k++) {
        FILE *o = outs[k]; if (!o) continue;
        fprintf(o, "\n=== ACC timing summary ===\n");
        fprintf(o, "%-16s %10s %8s %12s %12s %12s\n", "thread", "ticks", "misses",
                "max_jit_us", "mean_jit_us", "max_exec_us");
        for (int i = 0; i < NUM_SLOTS; i++) {
            stat_slot_t c = s->slot[i];
            if (c.name[0] == 0 || c.ticks == 0) continue;
            fprintf(o, "%-16s %10llu %8llu %12.1f %12.1f %12.1f\n", c.name,
                    (unsigned long long)c.ticks, (unsigned long long)c.misses,
                    c.max_jitter_ns / 1e3, (double)c.sum_jitter_ns / (double)c.ticks / 1e3,
                    c.max_exec_ns / 1e3);
        }
    }
    if (sum) fclose(sum);
    fclose(csv);
    return 0;
}
