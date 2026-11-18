# ACC dashboard tools

This folder contains terminal-based live dashboard tools for the ACC project.

If you prefer a native binary dashboard, use `acc_dash` in this folder to show the same live trace and safety information.

## Notes

- The project creates `logs/s1`, `logs/s2`, etc. from `run_acc.sh`.
- The simulator CSV file is the source for live state updates.
- Safety events are logged in `safety_monitor.log`.
