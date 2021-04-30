#include "include/paper_compare.h"
#include "include/sim.h"
#include "include/plotter.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <sstream>
#include <fstream>
using namespace std;

#define DATA_DIR string("../../data/")


void paper_compare(int num_beacons_to_deploy_start, int num_beacons_to_deploy_end, int num_runs_per_swarm_size,
                   double d_perf, double xi_bar, double neigh_threshold, double rC) {
    // Parameters
    int num_rays_per_range_sensor = 1;
    int agent_max_steps = 100000;

    double base_dt = 10e-3;
    double gain_factor = 1.5;
    double k_obs = 0.1;
    double force_saturation_limit = 4.0;
    double minimum_force_threshold = 0.01;

    // Xi model
    XiParams xi_params;
    xi_params.d_perf = d_perf;
    xi_params.xi_bar = xi_bar;
    xi_params.neigh_threshold = neigh_threshold;
    xi_params.d_none = d_perf +\
     (M_PI * (rC - d_perf)) / acos((2 * neigh_threshold / xi_bar) - 1);


    // Storage prefix
    stringstream ss;
    ss << "xi_bar_" << fixed << setprecision(2) << xi_params.xi_bar << \
          "_d_perf_" << fixed << setprecision(2) << xi_params.d_perf << \
          "_d_none_" << fixed << setprecision(2) << xi_params.d_none << \
          "_tau_xi_" << fixed << setprecision(2) << xi_params.neigh_threshold;
    
    string storage_prefix = ss.str();

    // Dir for plot storage
    string plot_general_name = "paper_uniformity_compare/configs/" + ss.str();

    // Uniformity storage
    json data_storage;

    // Initializing simulator with parameters
    for (int num_agents_to_deploy = num_beacons_to_deploy_start - 1; num_agents_to_deploy < num_beacons_to_deploy_end; num_agents_to_deploy++) {
        vector<double> uniformities = vector<double>(num_runs_per_swarm_size, 0);
        for (int i = 0; i < num_runs_per_swarm_size; i++) {
            Simulator simulator(
                base_dt,
                gain_factor,
                k_obs,
                Env::ten_by_ten,
                num_agents_to_deploy,
                num_rays_per_range_sensor,
                xi_params,
                force_saturation_limit,
                minimum_force_threshold,
                agent_max_steps,
                ExpVecType::NEIGH_INDUCED_RANDOM
            );


            // Run simulator
            simulator.simulate();
            uniformities[i] = simulator.get_uniformity_after_deploying_num_agents(num_agents_to_deploy);
            if (i == 0) {
                plot_config(simulator, false, plot_general_name);
            }
        }
        data_storage[to_string(num_agents_to_deploy + 1)] = uniformities;
    }
    
    std::ofstream o(
        DATA_DIR + storage_prefix + "_runs_per_swarm_size_" + to_string(num_runs_per_swarm_size) + ".json"
    );
    o << std::setw(4) << data_storage << std::endl;
}