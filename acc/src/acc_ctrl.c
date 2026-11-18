/*
 * acc_ctrl - the ACC brain (3 threads)
 *
 *   control_thread (P50, 20 ms) : reads the newest sensor frame, picks the mode,
 *                                 runs the control law, sends the command to
 *                                 actuator_proc, sends the heartbeat.
 *   state_thread   (P45, events): main thread. Serves driver commands from
 *                                 hmi_proc (enable, set speed, time-gap level).
 *   adapt_thread   (P30, 100 ms): adapts the time gap and the gain scale to
 *                                 speed and closing rate.
 *
 * The control thread never takes a lock. Driver settings and the adaptive
 * parameters reach it through lock-free atomics.
 */
#include "acc_common.h"

static _Atomic int    g_enabled   = 0;
static _Atomic int    g_gap_level = 2;
static _Atomic double g_vset      = 25.0;
static _Atomic double g_tau       = 1.5;
static _Atomic double g_gain      = 1.0;

static sensor_shm_t   *g_sensor;
static safety_state_t *g_safety;
static ctrl_shm_t     *g_status;

/* ------------------------------------------------------------------ */
static void *adapt_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_ADAPT);
    periodic_t p;
    periodic_setup(&p, 0, ADAPT_PERIOD_MS, SL_ADAPT, "adapt_thread");
    static const double base_tau[4] = {1.5, 1.0, 1.5, 2.0};   /* index = gap level 1..3 */
    double tau = 1.5;
    sensor_frame_t f; memset(&f, 0, sizeof f);

    for (;;) {
        periodic_wait(&p);
        sensor_frame_t tmp;
        if (sl_read(&g_sensor->seq, &g_sensor->f, &tmp, sizeof tmp) == 0) f = tmp;

        int lvl = atomic_load(&g_gap_level);
        if (lvl < 1) lvl = 1;
        if (lvl > 3) lvl = 3;
        double target = base_tau[lvl];

        /* faster driving needs a longer gap */
        target += 0.4 * clampd((f.ego_speed_mps - 15.0) / 15.0, 0.0, 1.0);
        /* a closing lead car needs extra margin */
        if (f.target_valid)
            target += 0.5 * clampd(-f.rel_speed_mps / 6.0, 0.0, 1.0);
        target = clampd(target, TAU_MIN_S, TAU_MAX_S);

        /* rate limit: at most 0.5 s per second so the gap never jumps */
        double step = 0.05;
        tau += clampd(target - tau, -step, step);
        atomic_store(&g_tau, tau);

        /* gain scheduling: be more responsive in stop-and-go */
        double gs = 1.0 + 0.5 * (1.0 - clampd(f.ego_speed_mps / 10.0, 0.0, 1.0));
        atomic_store(&g_gain, gs);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
static void *control_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_CONTROL);
    periodic_t p;
    periodic_setup(&p, 0, CONTROL_PERIOD_MS, SL_CONTROL, "control_thread");

    int coid_act = -1, coid_safety = -1;
    unsigned cycle = 0, seq = 0;
    int emerg = 0;
    int fails = 0;
    sensor_frame_t f; memset(&f, 0, sizeof f);

    for (;;) {
        periodic_wait(&p);

        /* connections are made lazily, about once a second, never blocking */
        if ((cycle++ % 50) == 0) {
            lazy_open(&coid_act, NAME_ACT);
            lazy_open(&coid_safety, NAME_SAFETY);
        }

        sensor_frame_t tmp;
        if (sl_read(&g_sensor->seq, &g_sensor->f, &tmp, sizeof tmp) == 0) f = tmp;
        uint64_t now = now_ns();
        int have_frame = (f.t_meas_ns != 0);
        int64_t age_ns = have_frame ? (int64_t)now - (int64_t)f.t_meas_ns : (int64_t)SENSOR_STALE_NS + 1;
        int stale = !have_frame || age_ns > (int64_t)SENSOR_STALE_NS;

        uint32_t faults = __atomic_load_n(&g_safety->fault_flags, __ATOMIC_ACQUIRE);
        int enabled     = atomic_load(&g_enabled);
        double vset     = atomic_load(&g_vset);
        double tau      = atomic_load(&g_tau);
        double gs       = atomic_load(&g_gain);

        int    mode = MODE_OFF, active = 0;
        double a = 0.0, d_des = D0_M + tau * f.ego_speed_mps, ttc = 1e9;

        if (!enabled) {
            mode = MODE_OFF; emerg = 0;
        } else if (faults != 0 || stale) {
            atomic_store(&g_enabled, 0);
            enabled = 0;
            mode = MODE_FAULT; emerg = 0;
        } else {
            active = 1;
            double v = f.ego_speed_mps;
            double a_speed = KV_SPEED * (vset - v);
            a = a_speed;
            mode = MODE_SPEED;

            if (f.target_valid && f.dist_m < RADAR_RANGE_M) {
                double d = f.dist_m, vrel = f.rel_speed_mps;
                int closing = (vrel < -0.1);
                if (closing) ttc = d / (-vrel);

                double a_follow = gs * (KD * (d - d_des) + KVREL * vrel);
                if (closing && d < d_des * 1.2) {          /* kinematic braking demand */
                    double a_req = -(vrel * vrel) / (2.0 * fmax(d - D_MIN_M, 0.5));
                    if (a_req < a_follow) a_follow = a_req;
                }
                if (a_follow < a_speed) { a = a_follow; mode = MODE_FOLLOW; }

                if (ttc < TTC_EMERG_S || (emerg && ttc < TTC_EMERG_EXIT_S)) emerg = 1;
                else emerg = 0;
                if (emerg) { a = A_EMERG; mode = MODE_EMERGENCY; }
            } else {
                emerg = 0;
            }
            if (mode != MODE_EMERGENCY) a = clampd(a, A_MIN_ACC, A_MAX_ACC);
        }

        /* command to the actuator (bounded wait: a stuck actuator must not stall us) */
        if (coid_act >= 0) {
            act_cmd_msg_t cmd; reply_t rep;
            cmd.type  = MSG_ACT_CMD;
            cmd.flags = (uint16_t)((active ? CMD_F_ACTIVE : 0) | (mode == MODE_EMERGENCY ? CMD_F_EMERGENCY : 0));
            cmd.seq   = ++seq;
            cmd.t_ns  = now;
            cmd.a_cmd = a;
            uint64_t to = 5000000ULL;                       /* 5 ms */
            TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &to, NULL);
            if (MsgSend(coid_act, &cmd, sizeof cmd, &rep, sizeof rep) < 0) {
                if (++fails >= 3) { ConnectDetach(coid_act); coid_act = -1; fails = 0; }
            } else {
                fails = 0;
            }
        }

        ctrl_status_t st;
        st.t_ns = now; st.mode = mode; st.enabled = enabled; st.a_cmd = a;
        st.tau = tau; st.d_des = d_des; st.ttc = ttc; st.v_set = vset;
        st.gain_scale = gs; st.gap_level = atomic_load(&g_gap_level); st.pad = 0;
        sl_write(&g_status->seq, &g_status->d, &st, sizeof st);

        if (coid_safety >= 0) {
            if (MsgSendPulse(coid_safety, SIGEV_PULSE_PRIO_INHERIT, PULSE_HB_CTRL, (int)seq) < 0) {
                ConnectDetach(coid_safety);
                coid_safety = -1;
            }
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
int main(void)
{
    g_sensor = shm_map(SHM_SENSOR,      sizeof(sensor_shm_t));
    g_safety = shm_map(SHM_SAFETY,      sizeof(safety_state_t));
    g_status = shm_map(SHM_CTRL_STATUS, sizeof(ctrl_shm_t));
    if (!g_sensor || !g_safety || !g_status) return 1;

    name_attach_t *att = name_attach(NULL, NAME_CTRL, 0);
    if (!att) { perror("name_attach acc_ctrl"); return 1; }

    pthread_t t1, t2;
    rt_spawn(&t1, control_thread, NULL);
    rt_spawn(&t2, adapt_thread, NULL);

    /* main thread = state_thread */
    rt_set_self(PRIO_STATE);
    for (;;) {
        rx_buf_t msg;
        int rcvid = MsgReceive(att->chid, &msg, sizeof msg, NULL);
        if (rcvid < 0) { if (errno == EINTR) continue; perror("state_thread MsgReceive"); break; }

        if (rcvid == 0) {                                   /* pulse */
            if (msg.pulse.code == _PULSE_CODE_DISCONNECT) ConnectDetach(msg.pulse.scoid);
            continue;
        }
        if (msg.type == _IO_CONNECT) { MsgReply(rcvid, EOK, NULL, 0); continue; }
        if (msg.type > _IO_BASE && msg.type <= _IO_MAX) { MsgError(rcvid, ENOSYS); continue; }

        reply_t rep; memset(&rep, 0, sizeof rep);
        switch (msg.type) {
        case MSG_SET_ENABLE:
            atomic_store(&g_enabled, msg.m.ival ? 1 : 0);
            printf("[acc_ctrl] ACC %s\n", msg.m.ival ? "enabled" : "disabled");
            break;
        case MSG_SET_SPEED:
            atomic_store(&g_vset, clampd(msg.m.dval, 5.0, 40.0));
            printf("[acc_ctrl] set speed %.1f m/s\n", clampd(msg.m.dval, 5.0, 40.0));
            break;
        case MSG_SET_GAP:
            atomic_store(&g_gap_level, (int)clampd(msg.m.ival, 1, 3));
            printf("[acc_ctrl] gap level %d\n", (int)clampd(msg.m.ival, 1, 3));
            break;
        default:
            rep.status = -1;
        }
        fflush(stdout);
        MsgReply(rcvid, EOK, &rep, sizeof rep);
    }
    return 0;
}
