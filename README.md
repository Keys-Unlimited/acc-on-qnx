# Intelligent Adaptive Cruise Control on QNX Neutrino

7 processes, 15 threads. Matches the methodology document.

| Process | Threads (priority, period) |
|---|---|
| `sensor_proc` | radar_thread (P40, 20 ms), wheel_thread (P40, 20 ms), fusion_thread (P42, 20 ms, main) |
| `acc_ctrl` | control_thread (P50, 20 ms), state_thread (P45, events, main), adapt_thread (P30, 100 ms) |
| `actuator_proc` | cmd_thread (P48, MsgReceive, main), output_thread (P48, 10 ms) |
| `safety_monitor` | watchdog_thread (P55, 10 ms, main), ttc_thread (P52, 20 ms) |
| `hmi_proc` | input_thread (P20, events, main), display_thread (P15, 100 ms) |
| `diag_proc` | diag_thread (P10, 100 ms, main) |
| `vehicle_sim` | plant_thread (P38, 10 ms, main), scenario_thread (P35, 50 ms) |

## Build (on the machine with the QNX SDP)

    source ~/qnx800/qnxsdp-env.sh        # or your qnx710 equivalent
    make                                  # x86_64 VM
    make TARGET=gcc_ntoaarch64le          # aarch64 target

Copy `bin/` and `scripts/` to the QNX target (same directory), then run **as root**
(needed for SCHED_FIFO priorities above the default limit).

## Run

    ./scripts/run_acc.sh 1 60             # scenario 1 for 60 s
    ./scripts/run_acc.sh 1 60 hog         # same, with two CPU-hog stress processes at P9

| # | Scenario | What it shows |
|---|---|---|
| 1 | lead_brake | following, hard braking by the lead, re-acceleration |
| 2 | cut_in | target jump at 20 m, filter re-initialisation, brake limit |
| 3 | stop_go | following down to standstill and away again |
| 4 | sensor_drop | radar silent at 20 s -> stale frame -> fault -> driver takes over |
| 5 | ctrl_hang | acc_ctrl frozen at 22 s (SIGSTOP) -> heartbeat lost -> fault; AEB backstop |

Interactive use (separate terminals): start the processes by hand in the order in
`run_acc.sh`, but run `./bin/hmi_proc` without `-a` and type `e` + Enter to engage.

## Outputs (logs/sN/)

- `sim_summary.txt`  collision, minimum gap, strongest decel, max and rms jerk, rms headway error, time per mode
- `diag_summary.txt` per-thread max/mean jitter, deadline misses, max execution time
- `trace.csv`        10 Hz trace for plots (speeds, gap vs desired gap, time gap, pedals, mode, fault flags)
- `diag.csv`         timing statistics every 100 ms

Use these for the Evaluate step: loop jitter and deadline misses with and without `hog`,
headway error and jerk, and the fixed-gap comparison (see Tuning).

## How it works

    vehicle_sim --/acc_raw--> sensor_proc --/acc_sensor--> acc_ctrl --MsgSend--> actuator_proc
         ^                        (Kalman)        |  \                                |
         |                                        |   +--/acc_ctrl_status--> hmi_proc |
         +----------------------/acc_act_out------+-----------------------------------+

    sensor_proc, acc_ctrl, actuator_proc --heartbeat pulses--> safety_monitor
    safety_monitor --/acc_safety_state (fault flags, warn level)--> everyone
    every periodic thread --/acc_stats--> diag_proc

- Shared memory between processes uses a seqlock: one writer, no mutex on the control path.
- Driver commands, heartbeats and actuator commands use QNX message passing and pulses.
  acc_ctrl bounds its MsgSend to the actuator with TimerTimeout (5 ms).
- The control law takes the smaller of a speed-control and a gap-control acceleration
  (constant time-gap policy), limited to +2.0 / -3.5 m/s^2. Emergency mode (TTC < 1.8 s)
  allows -6 m/s^2. The adaptive layer varies the time gap 0.8 to 2.5 s from the driver's gap
  level, ego speed and closing rate, and scales the gains up at low speed.
- Safety: a missing heartbeat (3 periods) or a sensor frame older than 250 ms latches a fault.
  acc_ctrl and actuator_proc then release the pedals. ttc_thread is an independent
  emergency-brake backstop (TTC < 1.0 s) that works even when acc_ctrl is dead.
  In the simulator a driver model takes over 1.5 s after a fault.
- AEB braking to zero throttle and full brake while ACC is off is intentional: it is the emergency backstop.

## Tuning knobs (all in `src/acc_common.h`)

`KD`, `KVREL`, `KV_SPEED` (control gains), `D0_M` (standstill gap), `A_MAX_ACC`/`A_MIN_ACC`,
`TTC_EMERG_S`, `JERK_*`, `TAU_MIN_S`/`TAU_MAX_S`, periods and priorities.
Time-gap rules are in `adapt_thread` (acc_ctrl.c). For a fixed-gap baseline, make
`adapt_thread` store a constant (for example 1.5) into `g_tau` and set the gain to 1.0.

## Moving to a Raspberry Pi later

Only `vehicle_sim` touches the vehicle boundary:
it writes `/acc_raw` (`raw_data_t`: radar distance and relative speed, wheel speed) and reads
`/acc_act_out` (`act_out_t`: throttle and brake). Replace it with the real sensor/actuator side
and the other six processes stay unchanged.

## Troubleshooting

- `could not set priority`: run as root.
- CPU hogs are intentionally kept below essential ACC threads (P9), while the scenario harness sits above them at P35 to avoid starvation on constrained VMs.
- `acc_ctrl not found` / name_open fails: start order matters only for the first second;
  `run_acc.sh` handles it. Check `ls /dev/name/local/`.
- Stale state between runs: `rm -f /dev/shmem/acc_*`.
- Many "FAULT" lines at start: the sim must be running; sensor_proc needs raw data before frames exist.
- Jitter numbers look huge in a VM: expected. Compare `hog` vs `nohog` runs rather than absolute values.
