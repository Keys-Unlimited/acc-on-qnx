/*
 * vehicle_sim - test harness that stands in for the real car (2 threads)
 *
 *   plant_thread    (P38, 10 ms): main thread. Integrates ego-vehicle and lead-car
 *       dynamics, reads the pedals from actuator_proc, publishes noisy radar and
 *       wheel-speed data every 20 ms. Includes a simple driver model that takes
 *       over 1.5 s after the safety monitor disengages ACC.
 *   scenario_thread (P25, 50 ms): scripted lead-car behaviour and a 10 Hz CSV trace.
 *
 * This is the only process that touches the "vehicle" boundary:
 *    writes  /acc_raw      (radar + wheel speed)
 *    reads   /acc_act_out  (throttle, brake)
 * so it can later be replaced by real sensors and actuators without touching
 * the other six processes.
 *
 * usage: vehicle_sim <scenario 1..5> [duration_s] [logdir]
 *   1 lead_brake   lead car brakes hard, then speeds up
 *   2 cut_in       slower car cuts in at 20 m, later leaves
 *   3 stop_go      lead car oscillates between 0 and 12 m/s
 *   4 sensor_drop  radar goes silent at t=20 s, lead brakes at t=30 s
 *   5 ctrl_hang    same traffic as 4; run_acc.sh freezes acc_ctrl at t=20 s
 */
#include "acc_common.h"

typedef struct {
    double t;
    double x_e, v_e, a_e;
    double x_l, v_l, a_l_cmd;
    int    lead_present;
    int    radar_dropout;
} world_t;

static world_t         W;
static pthread_mutex_t Wmx;
static volatile sig_atomic_t g_run = 1;
static int    g_scen = 1;
static double g_dur  = 60.0;
static const char *g_logdir = "./logs";

static raw_shm_t      *g_raw;
static act_shm_t      *g_act;
static ctrl_shm_t     *g_ctrl;
static safety_state_t *g_safety;

/* metrics (guarded by Wmx) */
static double m_min_gap = 1e9, m_min_a = 0, m_max_jerk = 0, m_jerk2 = 0, m_he2 = 0;
static unsigned long m_jn = 0, m_hen = 0;
static int    m_collision = 0;
static double m_impact = 0;
static double m_mode_time[5];

static void on_sig(int s) { (void)s; g_run = 0; }

static double gauss(unsigned *seed)
{
    double u1 = (rand_r(seed) + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (rand_r(seed) + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void scenario_init(int sc)
{
    memset(&W, 0, sizeof W);
    switch (sc) {
    case 1: W.v_e = 20; W.x_l = 60; W.v_l = 20; W.lead_present = 1; break;
    case 2: W.v_e = 25; W.lead_present = 0; break;
    case 3: W.v_e = 8;  W.x_l = 25; W.v_l = 8;  W.lead_present = 1; break;
    default: W.v_e = 22; W.x_l = 40; W.v_l = 22; W.lead_present = 1; break;
    }
}

/* ------------------------------------------------------------------ */
static void *scenario_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_SCENARIO);
    periodic_t p;
    periodic_setup(&p, 0, SCENARIO_PERIOD_MS, SL_SCENARIO, "scenario_thread");

    char path[256];
    snprintf(path, sizeof path, "%s/trace.csv", g_logdir);
    FILE *tr = fopen(path, "w");
    if (tr) fprintf(tr, "t,v_ego,v_lead,gap,d_des,tau,a_ego,throttle,brake,mode,fault,warn,ttc\n");
    unsigned n = 0;
    int cut_done = 0;

    while (g_run) {
        periodic_wait(&p);
        n++;
        pthread_mutex_lock(&Wmx);
        double t = W.t;
        switch (g_scen) {
        case 1:
            if (t < 10)       W.a_l_cmd = 0;
            else if (t < 35)  W.a_l_cmd = (W.v_l > 8.0) ? -3.0 : 0.0;
            else              W.a_l_cmd = (W.v_l < 22.0) ? 1.5 : 0.0;
            break;
        case 2:
            if (t >= 12 && !cut_done) {                          /* cut-in */
                cut_done = 1;
                W.lead_present = 1; W.x_l = W.x_e + 20.0; W.v_l = 18.0;
            }
            W.a_l_cmd = 0;
            if (t >= 40) W.lead_present = 0;                     /* lead leaves the lane */
            break;
        case 3: {
            double vt = 6.0 * (1.0 + sin(2.0 * M_PI * t / 40.0));
            W.a_l_cmd = clampd(1.5 * (vt - W.v_l), -3.5, 2.0);
            break; }
        case 4:
        case 5:
            if (t >= 20 && g_scen == 4) W.radar_dropout = 1;
            if (t < 30) W.a_l_cmd = 0;
            else        W.a_l_cmd = (W.v_l > 10.0) ? -3.0 : 0.0;
            break;
        }
        double gap = W.x_l - W.x_e, v_e = W.v_e, v_l = W.v_l, a_e = W.a_e;
        int present = W.lead_present;
        pthread_mutex_unlock(&Wmx);

        if (tr && (n % 2) == 0) {
            ctrl_status_t st; memset(&st, 0, sizeof st);
            act_out_t ao;     memset(&ao, 0, sizeof ao);
            sl_read(&g_ctrl->seq, &g_ctrl->d, &st, sizeof st);
            sl_read(&g_act->seq, &g_act->d, &ao, sizeof ao);
            fprintf(tr, "%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%s,%u,%u,%.2f\n",
                    t, v_e, present ? v_l : 0.0, present ? gap : 0.0, st.d_des, st.tau, a_e,
                    ao.throttle, ao.brake, mode_name(st.mode),
                    __atomic_load_n(&g_safety->fault_flags, __ATOMIC_RELAXED),
                    __atomic_load_n(&g_safety->warn_level, __ATOMIC_RELAXED),
                    st.ttc > 1e8 ? -1.0 : st.ttc);
            fflush(tr);
        }
    }
    if (tr) fclose(tr);
    return NULL;
}

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    if (argc > 1) g_scen = atoi(argv[1]);
    if (argc > 2) g_dur  = atof(argv[2]);
    if (argc > 3) g_logdir = argv[3];
    if (g_scen < 1 || g_scen > 5) { fprintf(stderr, "scenario must be 1..5\n"); return 1; }

    g_raw    = shm_map(SHM_RAW,         sizeof(raw_shm_t));
    g_act    = shm_map(SHM_ACT_OUT,     sizeof(act_shm_t));
    g_ctrl   = shm_map(SHM_CTRL_STATUS, sizeof(ctrl_shm_t));
    g_safety = shm_map(SHM_SAFETY,      sizeof(safety_state_t));
    if (!g_raw || !g_act || !g_ctrl || !g_safety) return 1;

    signal(SIGTERM, on_sig);
    signal(SIGINT, on_sig);

    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&Wmx, &ma);
    scenario_init(g_scen);

    pthread_t ts;
    rt_spawn(&ts, scenario_thread, NULL);

    rt_set_self(PRIO_PLANT);                                  /* main thread = plant_thread */
    periodic_t p;
    periodic_setup(&p, 0, PLANT_PERIOD_MS, SL_PLANT, "plant_thread");

    const double dt = PLANT_PERIOD_MS / 1000.0;
    unsigned seed = 12345, tick = 0;
    raw_data_t raw; memset(&raw, 0, sizeof raw);
    double a_prev = 0.0, fault_t0 = -1.0;
    double t_end = g_dur;

    printf("[sim] scenario %d, %.0f s, logs in %s\n", g_scen, g_dur, g_logdir);
    fflush(stdout);

    while (g_run) {
        periodic_wait(&p);
        tick++;
        uint64_t now = now_ns();

        /* pedals from the actuator (ignored if the actuator went silent) */
        act_out_t ao; double thr = 0, brk = 0;
        if (sl_read(&g_act->seq, &g_act->d, &ao, sizeof ao) == 0 && ao.t_ns != 0) {
            int64_t age_ao = (int64_t)now - (int64_t)ao.t_ns;
            if (age_ao < 200000000LL) { thr = ao.throttle; brk = ao.brake; }
        }
        uint32_t fault = __atomic_load_n(&g_safety->fault_flags, __ATOMIC_ACQUIRE);

        ctrl_status_t cs; int have_cs = (sl_read(&g_ctrl->seq, &g_ctrl->d, &cs, sizeof cs) == 0);

        pthread_mutex_lock(&Wmx);
        W.t += dt;
        if (W.t >= t_end) g_run = 0;

        /* driver model: takes over after the reaction time once ACC is disengaged */
        if (fault && fault_t0 < 0) fault_t0 = W.t;
        int driver = (fault_t0 >= 0) && (W.t - fault_t0 >= DRIVER_REACTION_S);

        double gap = W.x_l - W.x_e;
        double a_tgt = VEH_THR_MAX * thr - VEH_BRK_MAX * brk - drag_est(W.v_e);
        if (driver) {
            double ttc = 1e9;
            if (W.lead_present && W.v_e > W.v_l) ttc = gap / (W.v_e - W.v_l);
            a_tgt = (ttc < 3.5) ? -5.0 : -drag_est(W.v_e) + 0.0;
        }
        W.a_e += (a_tgt - W.a_e) * dt / VEH_TAU_ACT;
        W.v_e += W.a_e * dt;
        if (W.v_e < 0) { W.v_e = 0; if (W.a_e < 0) W.a_e = 0; }
        W.x_e += W.v_e * dt;

        W.v_l += W.a_l_cmd * dt;
        if (W.v_l < 0) W.v_l = 0;
        W.x_l += W.v_l * dt;

        gap = W.x_l - W.x_e;
        if (W.lead_present) {
            if (gap < m_min_gap) m_min_gap = gap;
            if (gap <= 0.0 && !m_collision) { m_collision = 1; m_impact = W.v_e - W.v_l; }
            if (gap <= 0.0) { W.v_e = W.v_l; W.x_e = W.x_l; }   /* no passing through */
        }
        if (W.t > 5.0) {                                       /* ignore start-up */
            double j = (W.a_e - a_prev) / dt;
            if (fabs(j) > m_max_jerk) m_max_jerk = fabs(j);
            m_jerk2 += j * j; m_jn++;
            if (W.a_e < m_min_a) m_min_a = W.a_e;
            if (have_cs && cs.mode >= 0 && cs.mode < 5) m_mode_time[cs.mode] += dt;
            if (have_cs && cs.mode == MODE_FOLLOW && W.lead_present) {
                double e = gap - cs.d_des; m_he2 += e * e; m_hen++;
            }
        }
        a_prev = W.a_e;

        /* publish raw sensor data every 20 ms (every 2nd plant tick) */
        if ((tick % 2) == 0) {
            raw.wheel_t_ns = now;
            raw.wheel_speed_mps = W.v_e + 0.05 * gauss(&seed);
            if (!W.radar_dropout) {
                raw.radar_t_ns = now;
                if (W.lead_present && gap > 0.5 && gap < RADAR_RANGE_M) {
                    raw.radar_valid   = 1;
                    raw.dist_m        = gap + 0.30 * gauss(&seed);
                    raw.rel_speed_mps = (W.v_l - W.v_e) + 0.15 * gauss(&seed);
                } else {
                    raw.radar_valid = 0; raw.dist_m = 0; raw.rel_speed_mps = 0;
                }
            }                                                   /* dropout: radar_t_ns stops advancing */
            sl_write(&g_raw->seq, &g_raw->d, &raw, sizeof raw);
        }
        pthread_mutex_unlock(&Wmx);
    }

    /* ---------------- summary ---------------- */
    pthread_mutex_lock(&Wmx);
    char path[256];
    snprintf(path, sizeof path, "%s/sim_summary.txt", g_logdir);
    FILE *sum = fopen(path, "w");
    FILE *outs[2] = { stdout, sum };
    for (int k = 0; k < 2; k++) {
        FILE *o = outs[k]; if (!o) continue;
        fprintf(o, "\n=== Scenario %d result ===\n", g_scen);
        fprintf(o, "collision            : %s", m_collision ? "YES" : "no");
        if (m_collision) fprintf(o, " (impact %.1f m/s)", m_impact);
        fprintf(o, "\nminimum gap          : %.1f m\n", m_min_gap > 1e8 ? -1.0 : m_min_gap);
        fprintf(o, "strongest decel      : %.2f m/s^2\n", m_min_a);
        fprintf(o, "max |jerk|           : %.2f m/s^3\n", m_max_jerk);
        fprintf(o, "rms jerk             : %.2f m/s^3\n", m_jn ? sqrt(m_jerk2 / m_jn) : 0.0);
        fprintf(o, "rms headway error    : %.2f m (FOLLOW mode only)\n", m_hen ? sqrt(m_he2 / m_hen) : 0.0);
        fprintf(o, "time in mode (s)     : OFF %.1f  SPEED %.1f  FOLLOW %.1f  EMERG %.1f  FAULT %.1f\n",
                m_mode_time[0], m_mode_time[1], m_mode_time[2], m_mode_time[3], m_mode_time[4]);
        fprintf(o, "final ego speed      : %.1f m/s\n", W.v_e);
    }
    if (sum) fclose(sum);
    pthread_mutex_unlock(&Wmx);
    g_run = 0;
    pthread_join(ts, NULL);
    return 0;
}
