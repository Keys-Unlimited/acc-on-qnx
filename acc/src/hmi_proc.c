/*
 * hmi_proc - driver interface (2 threads, low priority)
 *
 *   input_thread   (P20, event driven): main thread. Sends driver commands to
 *       acc_ctrl (enable, set speed, gap level) and reset requests to the safety
 *       monitor.
 *   display_thread (P15, 100 ms): prints a status line twice a second.
 *
 * usage:  hmi_proc                       interactive (type a command + Enter)
 *         hmi_proc -a VSET GAP DELAY     auto mode for scripted tests:
 *                                        after DELAY s set speed VSET (m/s),
 *                                        gap level GAP (1..3) and enable ACC.
 *
 * interactive keys:  e = enable/disable   + / - = set speed +-1 m/s
 *                    g = next gap level   r = reset faults   q = quit
 */
#include "acc_common.h"

static ctrl_shm_t      *g_status;
static sensor_shm_t    *g_sensor;
static safety_state_t  *g_safety;

static void *display_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_HMI_DISP);
    periodic_t p;
    periodic_setup(&p, 0, HMI_DISP_PERIOD_MS, SL_HMI_DISP, "display_thread");
    unsigned n = 0;
    for (;;) {
        periodic_wait(&p);
        if ((++n % 5) != 0) continue;

        ctrl_status_t st; sensor_frame_t f;
        if (sl_read(&g_status->seq, &g_status->d, &st, sizeof st) != 0) continue;
        if (sl_read(&g_sensor->seq, &g_sensor->f, &f, sizeof f) != 0) continue;
        uint32_t faults = __atomic_load_n(&g_safety->fault_flags, __ATOMIC_RELAXED);
        uint32_t warn   = __atomic_load_n(&g_safety->warn_level, __ATOMIC_RELAXED);

        printf("[hmi] %-9s v=%5.1f set=%4.1f gap=%6.1f/%5.1f tau=%.2f a=%5.2f%s%s\n",
               mode_name(st.mode), f.ego_speed_mps, st.v_set,
               f.target_valid ? f.dist_m : 0.0, st.d_des, st.tau, st.a_cmd,
               warn == 1 ? "  !! COLLISION WARNING" : (warn >= 2 ? "  !! EMERGENCY BRAKE" : ""),
               faults ? "  !! FAULT: TAKE OVER" : "");
        fflush(stdout);
    }
    return NULL;
}

static int send_ctrl(int coid, uint16_t type, int ival, double dval)
{
    acc_msg_t m; reply_t r;
    memset(&m, 0, sizeof m);
    m.type = type; m.ival = ival; m.dval = dval;
    if (MsgSend(coid, &m, sizeof m, &r, sizeof r) < 0) { perror("MsgSend"); return -1; }
    return r.status;
}

int main(int argc, char **argv)
{
    g_status = shm_map(SHM_CTRL_STATUS, sizeof(ctrl_shm_t));
    g_sensor = shm_map(SHM_SENSOR,      sizeof(sensor_shm_t));
    g_safety = shm_map(SHM_SAFETY,      sizeof(safety_state_t));
    if (!g_status || !g_sensor || !g_safety) return 1;

    int    auto_mode = 0, gap = 2;
    double vset = 25.0, delay = 3.0;
    if (argc >= 5 && strcmp(argv[1], "-a") == 0) {
        auto_mode = 1; vset = atof(argv[2]); gap = atoi(argv[3]); delay = atof(argv[4]);
    }

    pthread_t t;
    rt_spawn(&t, display_thread, NULL);
    rt_set_self(PRIO_HMI_IN);                                 /* main thread = input_thread */

    int coid_ctrl   = name_open_retry(NAME_CTRL, 50);
    int coid_safety = name_open_retry(NAME_SAFETY, 50);
    if (coid_ctrl < 0) { fprintf(stderr, "hmi: acc_ctrl not found\n"); return 1; }

    if (auto_mode) {
        usleep((useconds_t)(delay * 1e6));
        send_ctrl(coid_ctrl, MSG_SET_SPEED, 0, vset);
        send_ctrl(coid_ctrl, MSG_SET_GAP, gap, 0);
        send_ctrl(coid_ctrl, MSG_SET_ENABLE, 1, 0);
        for (;;) sleep(3600);                                 /* display_thread keeps running */
    }

    printf("hmi: e=enable/disable  +/-=speed  g=gap level  r=reset faults  q=quit  (then Enter)\n");
    int enabled = 0;
    char line[64];
    while (fgets(line, sizeof line, stdin)) {
        switch (line[0]) {
        case 'e': enabled = !enabled; send_ctrl(coid_ctrl, MSG_SET_ENABLE, enabled, 0); break;
        case '+': vset += 1.0; send_ctrl(coid_ctrl, MSG_SET_SPEED, 0, vset); break;
        case '-': vset -= 1.0; send_ctrl(coid_ctrl, MSG_SET_SPEED, 0, vset); break;
        case 'g': gap = gap % 3 + 1; send_ctrl(coid_ctrl, MSG_SET_GAP, gap, 0); break;
        case 'r': if (coid_safety >= 0) send_ctrl(coid_safety, MSG_RESET, 0, 0); break;
        case 'q': return 0;
        default: break;
        }
    }
    return 0;
}
