#include "env.h"
#include "sim.h"
#include "plotter.h"

#include <iostream>
#include <list>
using namespace std;

#include "Eigen/Dense"
namespace eig = Eigen;

eig::Vector2d get_force_from_beacon(double k_i, eig::Vector2d agent_pos, eig::Vector2d beacon_pos, double beacon_exploration_dir, double beacon_xi) {
    return -k_i * (agent_pos - (beacon_pos + beacon_xi * eig::Rotation2Dd(beacon_exploration_dir).toRotationMatrix() * eig::Vector2d::UnitX()));
}

int main() {

    // Creating rectangular environment
    eig::Matrix<double, 2, 4> m;
    m << -1, 49, 49, -1,
        -1, -1, 19, 19;
    list<eig::Matrix<double, 2, eig::Dynamic>> obstacles;
    obstacles.push_back(m);
    Env environment = Env(obstacles);

    // Initializing simulator with parameters
    int n_agents = 80;
    int num_rays_per_range_sensor = 1;
    int agent_max_steps = 100000;

    double base_dt = 10e-3;
    double gain_factor = 1.5;
    double k_obs = 0.1;
    double force_saturation_limit = 4.0;
    double minimum_force_threshold = 0.01;

    XiParams xi_params;
    xi_params.d_perf = 3;
    xi_params.d_none = 6;
    xi_params.xi_bar = 20;
    xi_params.neigh_treshold = 0.5;
    
    Simulator simulator(
        base_dt,
        gain_factor,
        k_obs,
        environment,
        n_agents,
        num_rays_per_range_sensor,
        xi_params,
        *get_force_from_beacon,
        force_saturation_limit,
        minimum_force_threshold,
        agent_max_steps,
        ExpVecType::NEIGH_INDUCED_RANDOM
    );

    // Run simulator
    simulator.simulate();

    // Plotting result of simulation
    string general_name = "sector_random_hallway__xploration";
    plot_uniformity_traj(simulator, general_name);
    plot_config(simulator, general_name);
    for (const pair<int, vector<eig::Vector2i>> & agent_id_loop_initiators_pair : simulator.agent_id_to_loop_initiators_map) {
        plot_single_beacon_traj(simulator, agent_id_loop_initiators_pair.first, true, true, general_name + "looping");
        plot_agent_neigh_traj(simulator, agent_id_loop_initiators_pair.first, general_name + "looping");
    }
    return 0;
}