/*
 * sensor_proc - sensor acquisition and fusion (3 threads)
 *
 *   radar_thread  (P40, 20 ms) : reads the radar part of the raw frame
 *   wheel_thread  (P40, 20 ms) : reads the wheel-speed part of the raw frame
 *   fusion_thread (P42, 20 ms) : main thread. Kalman-filters distance and
 *                                relative speed, publishes the sensor frame
 *                                in shared memory and sends a heartbeat.
 *
 * Inside this process the two acquisition threads hand data to the fusion
 * thread through a priority-inheriting mutex. Across processes the frame
 * goes out through a seqlock (no mutex on the control path).
 */
#include "acc_common.h"

typedef struct { uint64_t t_ns; double dist, rel; uint32_t valid; } radar_meas_t;
typedef struct { uint64_t t_ns; double speed; } wheel_meas_t;

static pthread_mutex_t g_mx;
static radar_meas_t    g_radar;
static wheel_meas_t    g_wheel;
static raw_shm_t      *g_raw;

/* ---------- 2-state Kalman filter: x = [distance, relative speed] ---------- */
typedef struct { double x[2]; double P[2][2]; int init; } kf_t;

#define KF_R_DIST   0.09     /* (0.3 m)^2    radar distance noise   */
#define KF_R_VREL   0.0225   /* (0.15 m/s)^2 radar rel. speed noise */
#define KF_SIGMA_A  2.0      /* m/s^2        unmodelled accel. of lead/ego */

static void kf_reset(kf_t *k, double d, double v)
{
    k->x[0] = d; k->x[1] = v;
    k->P[0][0] = 1.0; k->P[0][1] = 0.0;
    k->P[1][0] = 0.0; k->P[1][1] = 1.0;
    k->init = 1;
}

static void kf_predict(kf_t *k, double dt)
{
    double sa2 = KF_SIGMA_A * KF_SIGMA_A;
    double q00 = sa2 * dt * dt * dt * dt / 4.0;
    double q01 = sa2 * dt * dt * dt / 2.0;
    double q11 = sa2 * dt * dt;
    double p00 = k->P[0][0], p01 = k->P[0][1], p10 = k->P[1][0], p11 = k->P[1][1];

    k->x[0] += k->x[1] * dt;
    k->P[0][0] = p00 + dt * (p10 + p01) + dt * dt * p11 + q00;
    k->P[0][1] = p01 + dt * p11 + q01;
    k->P[1][0] = p10 + dt * p11 + q01;
    k->P[1][1] = p11 + q11;
}

static void kf_update(kf_t *k, double zd, double zv)
{
    double p00 = k->P[0][0], p01 = k->P[0][1], p10 = k->P[1][0], p11 = k->P[1][1];
    double s00 = p00 + KF_R_DIST, s01 = p01, s10 = p10, s11 = p11 + KF_R_VREL;
    double det = s00 * s11 - s01 * s10;
    if (fabs(det) < 1e-12) return;
    double i00 =  s11 / det, i01 = -s01 / det, i10 = -s10 / det, i11 = s00 / det;

    double k00 = p00 * i00 + p01 * i10, k01 = p00 * i01 + p01 * i11;
    double k10 = p10 * i00 + p11 * i10, k11 = p10 * i01 + p11 * i11;

    double y0 = zd - k->x[0], y1 = zv - k->x[1];
    k->x[0] += k00 * y0 + k01 * y1;
    k->x[1] += k10 * y0 + k11 * y1;

    k->P[0][0] = (1 - k00) * p00 - k01 * p10;
    k->P[0][1] = (1 - k00) * p01 - k01 * p11;
    k->P[1][0] = -k10 * p00 + (1 - k11) * p10;
    k->P[1][1] = -k10 * p01 + (1 - k11) * p11;
}

/* ---------- acquisition threads ---------- */
static void *radar_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_RADAR);
    periodic_t p;
    periodic_setup(&p, 0, SENSOR_PERIOD_MS, SL_RADAR, "radar_thread");
    for (;;) {
        periodic_wait(&p);
        raw_data_t r;
        if (sl_read(&g_raw->seq, &g_raw->d, &r, sizeof r) == 0) {
            pthread_mutex_lock(&g_mx);
            g_radar.t_ns  = r.radar_t_ns;
            g_radar.dist  = r.dist_m;
            g_radar.rel   = r.rel_speed_mps;
            g_radar.valid = r.radar_valid;
            pthread_mutex_unlock(&g_mx);
        }
    }
    return NULL;
}

static void *wheel_thread(void *arg)
{
    (void)arg;
    rt_set_self(PRIO_WHEEL);
    periodic_t p;
    periodic_setup(&p, 0, SENSOR_PERIOD_MS, SL_WHEEL, "wheel_thread");
    for (;;) {
        periodic_wait(&p);
        raw_data_t r;
        if (sl_read(&g_raw->seq, &g_raw->d, &r, sizeof r) == 0) {
            pthread_mutex_lock(&g_mx);
            g_wheel.t_ns  = r.wheel_t_ns;
            g_wheel.speed = r.wheel_speed_mps;
            pthread_mutex_unlock(&g_mx);
        }
    }
    return NULL;
}

/* ---------- main = fusion_thread ---------- */
int main(void)
{
    g_raw = shm_map(SHM_RAW, sizeof(raw_shm_t));
    sensor_shm_t *out = shm_map(SHM_SENSOR, sizeof(sensor_shm_t));
    if (!g_raw || !out) return 1;

    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&g_mx, &ma);

    pthread_t t1, t2;
    rt_spawn(&t1, radar_thread, NULL);
    rt_spawn(&t2, wheel_thread, NULL);

    rt_set_self(PRIO_FUSION);
    periodic_t p;
    periodic_setup(&p, 0, SENSOR_PERIOD_MS, SL_FUSION, "fusion_thread");

    const double dt = SENSOR_PERIOD_MS / 1000.0;
    kf_t kf; memset(&kf, 0, sizeof kf);
    uint64_t last_radar_t = 0;
    int miss = 0;
    double wbuf[3] = {0, 0, 0}; int wn = 0, wi = 0;
    uint32_t frame_no = 0;
    int coid_safety = -1;
    unsigned cycle = 0;

    for (;;) {
        periodic_wait(&p);

        radar_meas_t rm; wheel_meas_t wm;
        pthread_mutex_lock(&g_mx);
        rm = g_radar; wm = g_wheel;
        pthread_mutex_unlock(&g_mx);

        if (rm.t_ns != 0 && wm.t_ns != 0) {
            /* 3-sample moving average on wheel speed */
            wbuf[wi] = wm.speed; wi = (wi + 1) % 3; if (wn < 3) wn++;
            double ego = 0; for (int i = 0; i < wn; i++) ego += wbuf[i];
            ego /= wn;

            if (kf.init) kf_predict(&kf, dt);
            if (rm.t_ns != last_radar_t) {          /* new radar measurement */
                last_radar_t = rm.t_ns;
                if (rm.valid) {
                    miss = 0;
                    if (!kf.init || fabs(rm.dist - kf.x[0]) > CUTIN_JUMP_M)
                        kf_reset(&kf, rm.dist, rm.rel);        /* new target / cut-in */
                    else
                        kf_update(&kf, rm.dist, rm.rel);
                } else if (++miss >= 3) {
                    kf.init = 0;                              /* target lost */
                }
            }

            sensor_frame_t f;
            f.t_meas_ns     = (rm.t_ns < wm.t_ns) ? rm.t_ns : wm.t_ns;
            f.t_pub_ns      = now_ns();
            f.target_valid  = kf.init ? 1u : 0u;
            f.dist_m        = kf.init ? kf.x[0] : 0.0;
            f.rel_speed_mps = kf.init ? kf.x[1] : 0.0;
            f.ego_speed_mps = ego;
            f.frame_no      = ++frame_no;
            sl_write(&out->seq, &out->f, &f, sizeof f);
        }

        /* heartbeat to the safety monitor (lazy connect, retry about once a second) */
        if (coid_safety < 0 && (cycle++ % 50) == 0) lazy_open(&coid_safety, NAME_SAFETY);
        if (coid_safety >= 0) {
            if (MsgSendPulse(coid_safety, SIGEV_PULSE_PRIO_INHERIT, PULSE_HB_SENSOR, (int)frame_no) < 0) {
                ConnectDetach(coid_safety);
                coid_safety = -1;
            }
        }
    }
    return 0;
}
