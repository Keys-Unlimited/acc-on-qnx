#!/bin/sh
# Run one ACC test scenario on the QNX target.
#
#   ./scripts/run_acc.sh <scenario 1..5> [duration_s] [hog]
#
#   1 lead_brake  2 cut_in  3 stop_go  4 sensor_drop  5 ctrl_hang
#   "hog" adds two CPU-hog processes at priority 9 as a stress test.
#
# Results land in logs/s<scenario>/ :
#   sim_summary.txt   safety and comfort numbers (gap, decel, jerk, collision)
#   diag_summary.txt  per-thread jitter and deadline misses
#   trace.csv         10 Hz trace of speed, gap, time gap, pedals, mode
#   diag.csv          timing statistics every 100 ms
#   *.log             console output of each process

SC=${1:-1}
DUR=${2:-60}
HOG=${3:-nohog}
BIN=./bin
LOG=./logs/s$SC
mkdir -p $LOG

case $SC in
  1) VSET=30;;
  2) VSET=25;;
  3) VSET=15;;
  *) VSET=22;;
esac

rm -f /dev/shmem/acc_*

PIDS=""
start() {            # start <name> <args...>
  NAME=$1; shift
  $BIN/$NAME "$@" > $LOG/$NAME.log 2>&1 &
  LASTPID=$!
  PIDS="$PIDS $LASTPID"
}

start safety_monitor;  SAFETY_PID=$LASTPID
sleep 1
start actuator_proc
start acc_ctrl;        CTRL_PID=$LASTPID
start sensor_proc
start hmi_proc -a $VSET 2 4
start diag_proc $LOG;  DIAG_PID=$LASTPID
PIDS=${PIDS% $DIAG_PID}          # diag is stopped last, gracefully

if [ "$HOG" = "hog" ]; then
  $BIN/cpu_hog 9 & HOG1=$!
  $BIN/cpu_hog 9 & HOG2=$!
  PIDS="$PIDS $HOG1 $HOG2"
  echo "CPU hogs started (priority 9)"
fi
sleep 1

if [ "$SC" = "5" ]; then
  ( sleep 22; kill -STOP $CTRL_PID; echo "acc_ctrl frozen (SIGSTOP)" >> $LOG/inject.log ) &
fi

echo "Running scenario $SC for $DUR s ..."
$BIN/vehicle_sim $SC $DUR $LOG > $LOG/vehicle_sim.log 2>&1

# shut everything down
kill -CONT $CTRL_PID 2>/dev/null
sleep 1
kill -9 $PIDS 2>/dev/null
sleep 1
kill -TERM $DIAG_PID 2>/dev/null
sleep 1
rm -f /dev/shmem/acc_*

echo
cat $LOG/sim_summary.txt
cat $LOG/diag_summary.txt
echo
echo "Safety log:"; cat $LOG/safety_monitor.log
echo "Full results in $LOG/"
