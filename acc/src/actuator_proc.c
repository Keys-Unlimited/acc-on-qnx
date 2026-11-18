/*
 * actuator_proc - turns acceleration commands into throttle and brake (2 threads)
 *
 *   cmd_thread    (P48, MsgReceive): main thread. Receives commands from acc_ctrl,
 *                                    stores the newest one and replies at once.
 *   output_thread (P48, 10 ms)     : jerk-limits the request, maps it to throttle
 *                                    and brake with drag feed-forward, publishes
 *                                    the result for the vehicle and sends the
 *                                    heartbeat.
 *
 * Local fail-safe rules:
 *   - safety fault latched, or no command for 60 ms  -> release pedals
 *   - safety warn_level == 2 (AEB)                   -> full brake, whatever acc_ctrl says
 */
#include "acc_common.h"

typedef struct { uint64_t t_rx_ns; double a_cmd; uint32_t flags; uint32_t pad; } local_cmd_t;
typedef struct { uint32_t seq; uint32_t pad; local_cmd_t d; } local_cmd_shm_t;

static local_cmd_shm_t g_cmd;           /* process-local seqlock, writer = cmd_thread */
static act_shm_t      *g_out;
static sensor_shm_t   *g_sensor;
static safety_state_t *g_safety;

static void *output_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_ACT_OUT);
    periodic_t p;
    periodic_setup(&p, 0, ACT_PERIOD_MS, SL_ACT_OUT, "output_thread");

    const double dt = ACT_PERIOD_MS / 1000.0;
    double a_applied = 0.0;
    double v = 0.0;
    int aeb_latched = 0;
    int coid_safety = -1;
    unsigned cycle = 0, n = 0;
    local_cmd_t last_cmd; memset(&last_cmd, 0, sizeof last_cmd);

    for (;;) {
        periodic_wait(&p);

        local_cmd_t c; memset(&c, 0, sizeof c);
        int have_cmd = (sl_read(&g_cmd.seq, &g_cmd.d, &c, sizeof c) == 0);
        if (!have_cmd) c = last_cmd;
        else last_cmd = c;

        sensor_frame_t f; memset(&f, 0, sizeof f);
        if (sl_read(&g_sensor->seq, &g_sensor->f, &f, sizeof f) == 0) v = f.ego_speed_mps;
        uint64_t now = now_ns();

        uint32_t faults = __atomic_load_n(&g_safety->fault_flags, __ATOMIC_ACQUIRE);
        uint32_t warn   = __atomic_load_n(&g_safety->warn_level, __ATOMIC_ACQUIRE);
        int64_t age_ns  = (c.t_rx_ns != 0) ? (int64_t)now - (int64_t)c.t_rx_ns : (int64_t)CMD_STALE_NS + 1;
        int cmd_stale   = (c.t_rx_ns == 0) || age_ns > (int64_t)CMD_STALE_NS;

        double a_target, jerk = JERK_NORMAL;
        int active = 1;

        if (warn >= 2) aeb_latched = 1; else aeb_latched = 0;

        if (aeb_latched) {
            a_target = -VEH_BRK_MAX; jerk = JERK_AEB; active = 0;
        } else if (faults != 0 || cmd_stale || !(c.flags & CMD_F_ACTIVE)) {
            a_target = -drag_est(v); active = 0;             /* release pedals */
        } else {
            a_target = c.a_cmd;
            if (c.flags & CMD_F_EMERGENCY) jerk = JERK_EMERG;
        }

        double step = jerk * dt;
        a_applied += clampd(a_target - a_applied, -step, step);

        double a_net = a_applied + drag_est(v);              /* what the pedals must produce */
        double thr = 0.0, brk = 0.0;
        if (a_net > 0.0) thr = clampd(a_net / VEH_THR_MAX, 0.0, 1.0);
        else             brk = clampd(-a_net / VEH_BRK_MAX, 0.0, 1.0);

        act_out_t o;
        o.t_ns = now; o.throttle = thr; o.brake = brk; o.a_applied = a_applied;
        o.active = (uint32_t)active; o.aeb = (uint32_t)aeb_latched;
        sl_write(&g_out->seq, &g_out->d, &o, sizeof o);

        if (coid_safety < 0 && (cycle++ % 100) == 0) lazy_open(&coid_safety, NAME_SAFETY);
        if (coid_safety >= 0) {
            if (MsgSendPulse(coid_safety, SIGEV_PULSE_PRIO_INHERIT, PULSE_HB_ACT, (int)(++n)) < 0) {
                ConnectDetach(coid_safety);
                coid_safety = -1;
            }
        }
    }
    return NULL;
}

int main(void)
{
    g_out    = shm_map(SHM_ACT_OUT, sizeof(act_shm_t));
    g_sensor = shm_map(SHM_SENSOR,  sizeof(sensor_shm_t));
    g_safety = shm_map(SHM_SAFETY,  sizeof(safety_state_t));
    if (!g_out || !g_sensor || !g_safety) return 1;

    name_attach_t *att = name_attach(NULL, NAME_ACT, 0);
    if (!att) { perror("name_attach acc_actuator"); return 1; }

    pthread_t t;
    rt_spawn(&t, output_thread, NULL);

    rt_set_self(PRIO_ACT_CMD);                               /* main thread = cmd_thread */
    for (;;) {
        rx_buf_t msg;
        int rcvid = MsgReceive(att->chid, &msg, sizeof msg, NULL);
        if (rcvid < 0) { if (errno == EINTR) continue; perror("cmd_thread MsgReceive"); break; }
        if (rcvid == 0) {
            if (msg.pulse.code == _PULSE_CODE_DISCONNECT) ConnectDetach(msg.pulse.scoid);
            continue;
        }
        if (msg.type == _IO_CONNECT) { MsgReply(rcvid, EOK, NULL, 0); continue; }
        if (msg.type > _IO_BASE && msg.type <= _IO_MAX) { MsgError(rcvid, ENOSYS); continue; }

        reply_t rep; memset(&rep, 0, sizeof rep);
        if (msg.type == MSG_ACT_CMD) {
            local_cmd_t c;
            c.t_rx_ns = now_ns();           /* age is measured on our own clock reading */
            c.a_cmd   = msg.cmd.a_cmd;
            c.flags   = msg.cmd.flags;
            c.pad     = 0;
            sl_write(&g_cmd.seq, &g_cmd.d, &c, sizeof c);

            act_out_t o;
            if (sl_read(&g_out->seq, &g_out->d, &o, sizeof o) == 0) {
                rep.throttle = (float)o.throttle; rep.brake = (float)o.brake;
            }
        } else {
            rep.status = -1;
        }
        MsgReply(rcvid, EOK, &rep, sizeof rep);
    }
    return 0;
}
