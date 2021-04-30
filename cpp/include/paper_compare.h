#ifndef PAPER_COMPARE_H
#define PAPER_COMPARE_H

void paper_compare(
    int num_beacons_to_deploy_start,
    int num_beacons_to_deploy_end,
    int num_runs_per_swarm_size,
    double d_perf,
    double xi_bar,
    double neigh_threshold,
    double rC = 4
    );

#endif