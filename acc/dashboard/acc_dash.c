#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#define MAX_LINE 512

static void clear_screen(void)
{
    printf("\033[2J\033[H");
    fflush(stdout);
}

static void trim_newline(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) {
        s[--n] = '\0';
    }
}

static FILE *open_scenario_log(int scenario, const char *filename,
                              char *path_out, size_t path_size)
{
    snprintf(path_out, path_size, "./logs/s%d/%s", scenario, filename);
    FILE *f = fopen(path_out, "r");
    if (f) return f;

    snprintf(path_out, path_size, "../logs/s%d/%s", scenario, filename);
    f = fopen(path_out, "r");
    if (f) return f;

    path_out[0] = '\0';
    return NULL;
}

static int read_last_trace_line(int scenario, char *out, size_t out_size)
{
    char trace_path[256];
    FILE *f = open_scenario_log(scenario, "trace.csv", trace_path, sizeof(trace_path));
    if (!f) return -1;

    char line[MAX_LINE];
    out[0] = '\0';
    while (fgets(line, sizeof(line), f)) {
        trim_newline(line);
        if (strchr(line, ',') != NULL) {
            snprintf(out, out_size, "%s", line);
        }
    }
    fclose(f);
    return out[0] ? 0 : -1;
}

static void print_last_safety_lines(int scenario)
{
    char safe_path[256];
    FILE *f = open_scenario_log(scenario, "safety_monitor.log",
                                safe_path, sizeof(safe_path));
    if (!f) {
        printf("  Safety: waiting for safety_monitor.log\n");
        return;
    }

    char lines[5][MAX_LINE];
    int n = 0;
    while (fgets(lines[n], sizeof(lines[n]), f) && n < 5) {
        trim_newline(lines[n]);
        n++;
    }
    fclose(f);

    printf("  Safety log:\n");
    for (int i = 0; i < n; i++) {
        printf("    %s\n", lines[i]);
    }
    if (n == 0) {
        printf("    no safety events yet\n");
    }
}

static int split_csv_line(char *line, char **cols, int max_cols)
{
    int n = 0;
    char *p = line;
    while (n < max_cols) {
        cols[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

static void print_dashboard(int scenario)
{
    char line[MAX_LINE];
    char *cols[20];
    int ncols = 0;

    clear_screen();
    printf("===========================================================\n");
    printf("              ACC LIVE TERMINAL DASHBOARD\n");
    printf("===========================================================\n");

    if (read_last_trace_line(scenario, line, sizeof(line)) != 0) {
        printf("Scenario %d: waiting for trace.csv...\n", scenario);
        printf("Run: ./scripts/run_acc.sh %d 60\n", scenario);
        printf("===========================================================\n");
        return;
    }

    ncols = split_csv_line(line, cols, 20);
    if (ncols < 12) {
        printf("Trace format not ready yet.\n");
        printf("Last line: %s\n", line);
        printf("===========================================================\n");
        return;
    }

    double t = atof(cols[0]);
    double v_ego = atof(cols[1]);
    double v_lead = atof(cols[2]);
    double gap = atof(cols[3]);
    double d_des = atof(cols[4]);
    double tau = atof(cols[5]);
    double a_ego = atof(cols[6]);
    double throttle = atof(cols[7]);
    double brake = atof(cols[8]);
    int fault = atoi(cols[9]);
    int warn = atoi(cols[10]);
    double ttc = atof(cols[11]);

    printf("Time: %6.1f s   Scenario: %d   Faults: 0x%02x\n", t, scenario, fault);
    printf("Ego Speed: %6.1f m/s   Lead: %6.1f m/s   Gap: %6.1f m\n", v_ego, v_lead, gap);
    printf("Desired Gap: %6.1f m   TTC: %5.1f s   Tau: %4.2f s\n", d_des, ttc, tau);
    printf("Accel: %6.2f m/s^2   Throttle: %5.1f%%   Brake: %5.1f%%\n", a_ego, throttle, brake);
    printf("Warn Level: %d\n", warn);
    printf("-----------------------------------------------------------\n");
    print_last_safety_lines(scenario);
    printf("===========================================================\n");
    printf("Press Ctrl+C to exit\n");
}

int main(int argc, char **argv)
{
    int scenario = 1;
    if (argc > 1) {
        scenario = atoi(argv[1]);
        if (scenario < 1 || scenario > 5) {
            fprintf(stderr, "Usage: %s [scenario 1..5]\n", argv[0]);
            return 1;
        }
    }

    for (;;) {
        print_dashboard(scenario);
        usleep(500000);
    }

    return 0;
}
