/*
 * safety_monitor - supervises the ACC chain (2 threads)
 *
 *   watchdog_thread (P55, 10 ms): main thread. Owns the "acc_safety" channel,
 *       which receives both its own timer pulse and the heartbeat pulses from
 *       sensor_proc, acc_ctrl and actuator_proc. A missing heartbeat (3 periods)
 *       or a sensor frame older than 100 ms latches a fault. Faults make acc_ctrl
 *       and actuator_proc release the pedals and hand control back to the driver.
 *       Also serves MSG_RESET from the HMI.
 *   ttc_thread (P52, 20 ms): independent time-to-collision check on the sensor
 *       frame. Sets warn_level 1 (warning) or 2 (emergency brake backstop).
 */
#include "acc_common.h"

static safety_state_t *g_safety;
static sensor_shm_t   *g_sensor;

enum { HB_SENSOR, HB_CTRL, HB_ACT, NUM_HB };
static const uint64_t HB_TIMEOUT_NS[NUM_HB] = { 60000000ULL, 60000000ULL, 30000000ULL };
static const uint32_t HB_FAULT[NUM_HB]      = { FAULT_SENSOR_HB, FAULT_CTRL_HB, FAULT_ACT_HB };
static const char    *HB_NAME[NUM_HB]       = { "sensor_proc", "acc_ctrl", "actuator_proc" };

static void *ttc_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_TTC);
    periodic_t p;
    periodic_setup(&p, 0, TTC_PERIOD_MS, SL_TTC, "ttc_thread");
    sensor_frame_t f; memset(&f, 0, sizeof f);
    uint32_t level = 0;

    for (;;) {
        periodic_wait(&p);
        sensor_frame_t tmp;
        if (sl_read(&g_sensor->seq, &g_sensor->f, &tmp, sizeof tmp) == 0) f = tmp;
        uint64_t now = now_ns();

        int64_t age_ns = (f.t_meas_ns != 0) ? (int64_t)now - (int64_t)f.t_meas_ns : (int64_t)SENSOR_STALE_NS + 1;
        int usable = f.t_meas_ns != 0 && age_ns <= (int64_t)SENSOR_STALE_NS && f.target_valid;
        double ttc = 1e9;
        if (usable && f.rel_speed_mps < -0.1) ttc = f.dist_m / (-f.rel_speed_mps);

        uint32_t nl = 0;
        if (ttc < WARN_TTC_S) nl = 1;
        if (ttc < AEB_TTC_S || (level == 2 && ttc < AEB_EXIT_TTC_S)) nl = 2;
        if (nl != level) {
            level = nl;
            __atomic_store_n(&g_safety->warn_level, level, __ATOMIC_RELEASE);
        }
    }
    return NULL;
}

int main(void)
{
    g_safety = shm_map(SHM_SAFETY, sizeof(safety_state_t));
    g_sensor = shm_map(SHM_SENSOR, sizeof(sensor_shm_t));
    if (!g_safety || !g_sensor) return 1;

    name_attach_t *att = name_attach(NULL, NAME_SAFETY, 0);
    if (!att) { perror("name_attach acc_safety"); return 1; }

    pthread_t t;
    rt_spawn(&t, ttc_thread, NULL);

    rt_set_self(PRIO_WATCHDOG);                              /* main thread = watchdog_thread */
    periodic_t p;
    periodic_setup(&p, att->chid, WATCHDOG_PERIOD_MS, SL_WATCHDOG, "watchdog_thread");

    uint64_t last_hb[NUM_HB] = {0, 0, 0};
    int armed[NUM_HB] = {0, 0, 0};

    for (;;) {
        rx_buf_t msg;
        int rcvid = MsgReceive(att->chid, &msg, sizeof msg, NULL);
        if (rcvid < 0) { if (errno == EINTR) continue; perror("watchdog MsgReceive"); break; }

        if (rcvid == 0) {                                    /* pulses */
            uint64_t now = now_ns();
            switch (msg.pulse.code) {
            case PULSE_HB_SENSOR: last_hb[HB_SENSOR] = now; armed[HB_SENSOR] = 1; break;
            case PULSE_HB_CTRL:   last_hb[HB_CTRL]   = now; armed[HB_CTRL]   = 1; break;
            case PULSE_HB_ACT:    last_hb[HB_ACT]    = now; armed[HB_ACT]    = 1; break;
            case PULSE_TICK: {
                periodic_on_tick(&p);
                uint32_t faults = __atomic_load_n(&g_safety->fault_flags, __ATOMIC_RELAXED);
                uint32_t nf = faults;
                for (int i = 0; i < NUM_HB; i++) {
                    int64_t age_hb = (int64_t)now - (int64_t)last_hb[i];
                    if (armed[i] && age_hb > (int64_t)HB_TIMEOUT_NS[i]) nf |= HB_FAULT[i];
                }

                sensor_frame_t f; memset(&f, 0, sizeof f);
                if (sl_read(&g_sensor->seq, &g_sensor->f, &f, sizeof f) == 0 &&
                    f.t_meas_ns != 0) {
                    int64_t age_ns = (int64_t)now - (int64_t)f.t_meas_ns;
                    if (age_ns > (int64_t)SENSOR_STALE_NS) nf |= FAULT_SENSOR_STALE;
                }

                if (nf != faults) {
                    __atomic_store_n(&g_safety->fault_flags, nf, __ATOMIC_RELEASE);
                    printf("[safety] FAULT latched: flags=0x%02x ->", nf);
                    for (int i = 0; i < NUM_HB; i++) if (nf & HB_FAULT[i]) printf(" %s heartbeat lost;", HB_NAME[i]);
                    if (nf & FAULT_SENSOR_STALE) printf(" sensor frame stale;");
                    printf(" ACC disengaged, driver must take over\n");
                    fflush(stdout);
                }
                periodic_done(&p);
                break; }
            case _PULSE_CODE_DISCONNECT: ConnectDetach(msg.pulse.scoid); break;
            default: break;
            }
            continue;
        }

        if (msg.type == _IO_CONNECT) { MsgReply(rcvid, EOK, NULL, 0); continue; }
        if (msg.type > _IO_BASE && msg.type <= _IO_MAX) { MsgError(rcvid, ENOSYS); continue; }

        reply_t rep; memset(&rep, 0, sizeof rep);
        if (msg.type == MSG_RESET) {
            sensor_frame_t f; memset(&f, 0, sizeof f);
            if (sl_read(&g_sensor->seq, &g_sensor->f, &f, sizeof f) == 0) {
                /* keep snapshot stable for the reset check */
            }
            uint64_t now = now_ns();
            int healthy = 1;
            for (int i = 0; i < NUM_HB; i++) {
                int64_t age_hb = (int64_t)now - (int64_t)last_hb[i];
                if (!armed[i] || age_hb > (int64_t)HB_TIMEOUT_NS[i]) healthy = 0;
            }
            if (f.t_meas_ns == 0 || (int64_t)now - (int64_t)f.t_meas_ns > (int64_t)SENSOR_STALE_NS) healthy = 0;
            if (healthy) {
                __atomic_store_n(&g_safety->fault_flags, 0u, __ATOMIC_RELEASE);
                __atomic_store_n(&g_safety->reset_count, g_safety->reset_count + 1, __ATOMIC_RELAXED);
                printf("[safety] faults cleared by driver reset\n");
            } else {
                rep.status = -1;
                printf("[safety] reset refused: system not healthy\n");
            }
            fflush(stdout);
        } else {
            rep.status = -1;
        }
        MsgReply(rcvid, EOK, &rep, sizeof rep);
    }
    return 0;
}
