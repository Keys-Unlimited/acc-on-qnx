/*
 * acc_common.h - shared definitions for the Intelligent Adaptive Cruise Control
 * (ACC) project on QNX Neutrino.
 *
 * Everything that two or more processes must agree on lives here:
 * names, priorities, periods, shared-memory layouts, message formats,
 * control parameters and vehicle-model constants.
 */
#ifndef ACC_COMMON_H
#define ACC_COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/neutrino.h>
#include <sys/iomsg.h>
#include <sys/dispatch.h>

/* ------------------------------------------------------------------ */
/* Names                                                              */
/* ------------------------------------------------------------------ */
#define NAME_CTRL        "acc_ctrl"          /* name_attach: acc_ctrl state_thread   */
#define NAME_ACT         "acc_actuator"      /* name_attach: actuator cmd_thread     */
#define NAME_SAFETY      "acc_safety"        /* name_attach: safety watchdog_thread  */

#define SHM_RAW          "/acc_raw"          /* vehicle_sim  -> sensor_proc  */
#define SHM_SENSOR       "/acc_sensor"       /* sensor_proc  -> acc_ctrl, safety, actuator */
#define SHM_ACT_OUT      "/acc_act_out"      /* actuator     -> vehicle_sim  */
#define SHM_CTRL_STATUS  "/acc_ctrl_status"  /* acc_ctrl     -> hmi, vehicle_sim */
#define SHM_SAFETY       "/acc_safety_state" /* safety       -> everyone     */
#define SHM_STATS        "/acc_stats"        /* every thread -> diag_proc    */

/* ------------------------------------------------------------------ */
/* Thread priorities (QNX: higher number = higher priority)           */
/* ------------------------------------------------------------------ */
#define PRIO_WATCHDOG    55
#define PRIO_TTC         52
#define PRIO_CONTROL     50
#define PRIO_ACT_CMD     48
#define PRIO_ACT_OUT     48
#define PRIO_STATE       45
#define PRIO_FUSION      42
#define PRIO_RADAR       40
#define PRIO_WHEEL       40
#define PRIO_PLANT       38
#define PRIO_ADAPT       30
#define PRIO_SCENARIO    35
#define PRIO_HMI_IN      20
#define PRIO_HMI_DISP    15
#define PRIO_DIAG        10
#define PRIO_HOG         9

/* ------------------------------------------------------------------ */
/* Periods (ms)                                                       */
/* ------------------------------------------------------------------ */
#define SENSOR_PERIOD_MS   20
#define CONTROL_PERIOD_MS  20
#define ADAPT_PERIOD_MS    100
#define ACT_PERIOD_MS      10
#define WATCHDOG_PERIOD_MS 10
#define TTC_PERIOD_MS      20
#define HMI_DISP_PERIOD_MS 100
#define DIAG_PERIOD_MS     100
#define PLANT_PERIOD_MS    10
#define SCENARIO_PERIOD_MS 50

/* ------------------------------------------------------------------ */
/* Pulse codes and message types                                      */
/* ------------------------------------------------------------------ */
#define PULSE_TICK       (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_HB_SENSOR  (_PULSE_CODE_MINAVAIL + 2)
#define PULSE_HB_CTRL    (_PULSE_CODE_MINAVAIL + 3)
#define PULSE_HB_ACT     (_PULSE_CODE_MINAVAIL + 4)

#define MSG_SET_ENABLE   (_IO_MAX + 1)   /* ival = 0/1            */
#define MSG_SET_SPEED    (_IO_MAX + 2)   /* dval = m/s            */
#define MSG_SET_GAP      (_IO_MAX + 3)   /* ival = 1..3           */
#define MSG_RESET        (_IO_MAX + 4)   /* clear latched faults  */
#define MSG_ACT_CMD      (_IO_MAX + 5)   /* acc_ctrl -> actuator  */

typedef struct { uint16_t type; uint16_t flags; int32_t ival; double dval; } acc_msg_t;

#define CMD_F_ACTIVE     0x1u
#define CMD_F_EMERGENCY  0x2u
typedef struct {
    uint16_t type; uint16_t flags; uint32_t seq;
    uint64_t t_ns;
    double   a_cmd;                 /* requested acceleration, m/s^2 */
} act_cmd_msg_t;

typedef struct { int32_t status; float throttle; float brake; } reply_t;

typedef union {
    uint16_t       type;
    struct _pulse  pulse;
    acc_msg_t      m;
    act_cmd_msg_t  cmd;
} rx_buf_t;

/* ------------------------------------------------------------------ */
/* Seqlock helpers: one writer, many readers, no mutex.               */
/* The writer bumps seq to odd, writes, bumps to even. A reader       */
/* retries if seq was odd or changed while it copied.                 */
/* ------------------------------------------------------------------ */
static inline void sl_write(uint32_t *seq, void *dst, const void *src, size_t n)
{
    uint32_t s = __atomic_load_n(seq, __ATOMIC_RELAXED);
    __atomic_store_n(seq, s + 1, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_RELEASE);
    memcpy(dst, src, n);
    __atomic_store_n(seq, s + 2, __ATOMIC_RELEASE);
}

static inline int sl_read(const uint32_t *seq, const void *src, void *dst, size_t n)
{
    for (int i = 0; i < 16; i++) {
        uint32_t s1 = __atomic_load_n(seq, __ATOMIC_ACQUIRE);
        if (s1 & 1u) continue;
        memcpy(dst, src, n);
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        uint32_t s2 = __atomic_load_n(seq, __ATOMIC_RELAXED);
        if (s1 == s2) return 0;
    }
    return -1;
}

/* ------------------------------------------------------------------ */
/* Shared-memory layouts                                              */
/* ------------------------------------------------------------------ */
typedef struct {                       /* raw sensor data from the (simulated) car */
    uint64_t radar_t_ns;
    double   dist_m;
    double   rel_speed_mps;            /* lead speed minus ego speed */
    uint32_t radar_valid;              /* 1 = a target is being tracked */
    uint32_t pad;
    uint64_t wheel_t_ns;
    double   wheel_speed_mps;
} raw_data_t;
typedef struct { uint32_t seq; uint32_t pad; raw_data_t d; } raw_shm_t;

typedef struct {                       /* filtered frame published by sensor_proc */
    uint64_t t_meas_ns;                /* time of the oldest measurement used     */
    uint64_t t_pub_ns;
    double   dist_m;
    double   rel_speed_mps;
    double   ego_speed_mps;
    uint32_t target_valid;
    uint32_t frame_no;
} sensor_frame_t;
typedef struct { uint32_t seq; uint32_t pad; sensor_frame_t f; } sensor_shm_t;

typedef struct {                       /* what the actuator actually applied */
    uint64_t t_ns;
    double   throttle;                 /* 0..1 */
    double   brake;                    /* 0..1 */
    double   a_applied;                /* m/s^2 requested after jerk limit */
    uint32_t active;                   /* 1 = ACC is driving the pedals */
    uint32_t aeb;                      /* 1 = emergency brake backstop active */
} act_out_t;
typedef struct { uint32_t seq; uint32_t pad; act_out_t d; } act_shm_t;

enum { MODE_OFF = 0, MODE_SPEED, MODE_FOLLOW, MODE_EMERGENCY, MODE_FAULT };
typedef struct {
    uint64_t t_ns;
    int32_t  mode;
    int32_t  enabled;
    double   a_cmd;
    double   tau;                      /* adaptive time gap, s */
    double   d_des;                    /* desired gap, m */
    double   ttc;                      /* time to collision, s (1e9 = none) */
    double   v_set;
    double   gain_scale;
    int32_t  gap_level;
    int32_t  pad;
} ctrl_status_t;
typedef struct { uint32_t seq; uint32_t pad; ctrl_status_t d; } ctrl_shm_t;

#define FAULT_SENSOR_HB    0x01u
#define FAULT_CTRL_HB      0x02u
#define FAULT_ACT_HB       0x04u
#define FAULT_SENSOR_STALE 0x08u
typedef struct {                       /* single aligned words, accessed with __atomic */
    uint32_t fault_flags;              /* latched, written by watchdog_thread */
    uint32_t warn_level;               /* 0 none, 1 warn, 2 AEB; written by ttc_thread */
    uint32_t reset_count;
    uint32_t pad;
} safety_state_t;

enum { SL_RADAR, SL_WHEEL, SL_FUSION, SL_CONTROL, SL_ADAPT, SL_ACT_OUT,
       SL_WATCHDOG, SL_TTC, SL_HMI_DISP, SL_DIAG, SL_PLANT, SL_SCENARIO,
       NUM_SLOTS };
typedef struct {                       /* single writer: the owning thread */
    char     name[24];
    uint64_t ticks;
    uint64_t misses;                   /* work finished after the next tick was due */
    int64_t  max_jitter_ns;            /* |wake time - ideal tick time| */
    uint64_t sum_jitter_ns;
    int64_t  max_exec_ns;              /* ClockCycles-measured execution time */
    uint64_t sum_exec_ns;
} stat_slot_t;
typedef struct { uint32_t nslots; uint32_t pad; stat_slot_t slot[NUM_SLOTS]; } stats_shm_t;

/* ------------------------------------------------------------------ */
/* ACC control parameters (tune here)                                 */
/* ------------------------------------------------------------------ */
#define D0_M               5.0     /* standstill gap */
#define D_MIN_M            3.0     /* braking-distance reference */
#define KD                 0.25    /* gap error gain          [1/s^2] */
#define KVREL              0.90    /* relative speed gain     [1/s]   */
#define KV_SPEED           0.40    /* speed error gain        [1/s]   */
#define A_MAX_ACC          2.0     /* m/s^2 */
#define A_MIN_ACC         (-3.5)
#define A_EMERG           (-6.0)
#define TTC_EMERG_S        1.8
#define TTC_EMERG_EXIT_S   3.0
#define RADAR_RANGE_M      150.0
#define SENSOR_STALE_NS    250000000ULL   /* allow 3-stage 20 ms pipeline + VM jitter */
#define CMD_STALE_NS       60000000ULL    /* actuator: no command for 60 ms */
#define CUTIN_JUMP_M       6.0
#define TAU_MIN_S          0.8
#define TAU_MAX_S          2.5
#define JERK_NORMAL        2.5     /* m/s^3 */
#define JERK_EMERG         15.0
#define JERK_AEB           30.0
#define WARN_TTC_S         2.5
#define AEB_TTC_S          1.0
#define AEB_EXIT_TTC_S     1.5

/* ------------------------------------------------------------------ */
/* Vehicle model (used by vehicle_sim and, as feed-forward, by actuator) */
/* ------------------------------------------------------------------ */
#define VEH_THR_MAX        3.5     /* m/s^2 at full throttle */
#define VEH_BRK_MAX        9.0     /* m/s^2 at full brake    */
#define VEH_TAU_ACT        0.25    /* actuation lag, s       */
#define DRIVER_REACTION_S  1.5

static inline double drag_est(double v)
{
    return (v > 0.2) ? (0.10 + 0.0004 * v * v) : 0.0;
}
static inline double clampd(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ------------------------------------------------------------------ */
/* Utilities (acc_util.c)                                             */
/* ------------------------------------------------------------------ */
uint64_t       now_ns(void);                 /* CLOCK_MONOTONIC, system wide */
uint64_t       cyc_now(void);                /* ClockCycles()                */
double         cyc_to_ns(uint64_t cycles);
void          *shm_map(const char *name, size_t size);
int            name_open_retry(const char *name, int tries);
int            lazy_open(int *coid, const char *name);   /* one attempt, returns coid or -1 */
int            rt_set_self(int prio);
int            rt_spawn(pthread_t *t, void *(*fn)(void *), void *arg);
const char    *mode_name(int mode);
stats_shm_t   *stats_attach(void);

typedef struct {
    int       chid, coid;
    timer_t   tid;
    uint64_t  period_ns, t0_ns, k, exp_ns, cyc_wake;
    int       started;
    stat_slot_t *st;
} periodic_t;

/* chid == 0: create a private channel. Otherwise the timer pulse is delivered
 * to the given channel and the caller must call periodic_on_tick() itself. */
int  periodic_setup(periodic_t *p, int chid, unsigned period_ms, int slot, const char *name);
void periodic_wait(periodic_t *p);           /* ends previous cycle, sleeps, starts next */
void periodic_on_tick(periodic_t *p);        /* a PULSE_TICK was received */
void periodic_done(periodic_t *p);           /* work for this cycle finished */

#endif
