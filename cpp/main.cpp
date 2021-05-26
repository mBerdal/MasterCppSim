#include "include/env.h"
#include "include/sim.h"
#include "include/plotter.h"

#include "include/paper_compare.h"


#include <Eigen/Dense>
namespace eig = Eigen;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <list>
#include <fstream>
#include <iostream>
using namespace std;

int main() {
    double d_perf = 2;
    double xi_bar = 1;
    double tau_xi = 0.5;
    paper_compare(50, 100, d_perf, xi_bar, tau_xi);
    return 0;

    Env environment = Env::ten_by_ten;

    // Parameters
    int num_rays_per_range_sensor = 1;
    int agent_max_steps = 100000;

    double base_dt = 10e-3;
    double gain_factor = 1.5;
    double k_obs = 0.1;
    double force_saturation_limit = 4.0;
    double minimum_force_threshold = 0.01;

    // Parameters for xi-model
    XiParams xi_params;
    xi_params.d_perf = 3;
    xi_params.xi_bar = 20;
    xi_params.agent_neigh_threshold = 0.5;
    xi_params.beacon_neigh_threshold = 0.5;
    xi_params.d_none = 6;

    vector<int> n_agents_array = {3};
    for (const int & n_agents : n_agents_array) {
        Simulator sim = Simulator(
            base_dt,
            gain_factor,
            k_obs,
            Env::ten_by_ten,
            n_agents,
            num_rays_per_range_sensor,
            xi_params,
            force_saturation_limit,
            minimum_force_threshold,
            agent_max_steps,
            ExpVecType::NEIGH_INDUCED_RANDOM
        );

        string simulation_base_name = "try_4";
        sim.simulate();
        plot_config(sim, false, simulation_base_name);
        plot_uniformity_traj(sim, false, simulation_base_name);
        plot_single_beacon_traj(sim, 1, false, true, simulation_base_name);

        sim.save_to_json("test/", "test_1");

    }


    return 0;
}