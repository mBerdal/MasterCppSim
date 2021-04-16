#include "env.h"
#include "sim.h"
#include "plotter.h"
#include <iostream>
#include <list>

using namespace std;

Vector2d get_force_from_beacon(Vector2d agent_pos, int beacon_id, Vector2d beacon_pos, Vector2d beacon_exploration_dir, double beacon_xi) {
    double k_i = beacon_id + 1;
    double a_i = 1;
    return -k_i * (agent_pos - a_i * (beacon_pos + beacon_xi * beacon_exploration_dir));
}

int main() {
    int n_agents = 60;
    int num_rays_per_range_sensor = 1;

    // Creating rectangular environment
    Matrix<double, 2, 4> m;
    m << -1, 10, 10, -1,
        -1, -1, 10, 10;
    list<Matrix<double, 2, Dynamic>> obstacles;
    obstacles.push_back(m);
    Env environment = Env(obstacles);

    // Initializing simulator and running
    double base_dt = 10e-3;
    double k_obs = 1;
    
    Simulator simulator(
        base_dt,
        k_obs,
        environment,
        n_agents,
        num_rays_per_range_sensor,
        *get_force_from_beacon,
        4.0,
        0.01,
        100000
    );
    simulator.simulate();

    // Plotting final configuration
    plot_config(simulator, to_string(n_agents) + "_agnt_rand_square_world");

    return 0;
}