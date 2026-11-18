# ACC dashboard tools

This folder contains a minimal live dashboard for the ACC project.

## Terminal dashboard

Use from the project root on the Pi or target:

    python3 dashboard/terminal_dashboard.py 1

This reads the latest row in `logs/s1/trace.csv` and refreshes a simple terminal dashboard.

## Web dashboard

Install dependencies:

    python3 -m pip install -r dashboard/requirements.txt

Run the dashboard server:

    python3 dashboard/web_dashboard.py

Then open:

    http://<pi-ip>:8000/

The dashboard expects the ACC logs to be under the project `logs/` directory.

## Notes

- The project creates `logs/s1`, `logs/s2`, etc. from `run_acc.sh`.
- The simulator CSV file is the best source for live plots and state updates.
- The project already logs safety events in `safety_monitor.log`.
