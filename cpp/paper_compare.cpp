#include "include/paper_compare.h"
#include "include/sim.h"
#include "include/plotter.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <sstream>
#include <fstream>
using namespace std;

void paper_compare(int num_beacons_to_deploy, int num_runs,
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
    xi_params.agent_neigh_threshold = neigh_threshold;
    xi_params.beacon_neigh_threshold = neigh_threshold;
    xi_params.d_none = d_perf +\
     (M_PI * (rC - d_perf)) / acos((2 * neigh_threshold / xi_bar) - 1);


    // Storage prefix
    stringstream ss;
    ss << "xi_bar_" << fixed << setprecision(2) << xi_params.xi_bar << \
          "_d_perf_" << fixed << setprecision(2) << xi_params.d_perf << \
          "_d_none_" << fixed << setprecision(2) << xi_params.d_none << \
          "_tau_xi_" << fixed << setprecision(2) << xi_params.agent_neigh_threshold;
    
    string storage_prefix = ss.str();

    // Dir for plot storage
    string plot_general_name = "paper_uniformity_compare/configs/" + ss.str();

    // Uniformity storage
    json data_storage;

    // Initializing simulator with parameters
    for (int i = 0; i < num_runs; i++) {
        Simulator simulator(
            base_dt,
            gain_factor,
            k_obs,
            Env::ten_by_ten,
            num_beacons_to_deploy - 1,
            num_rays_per_range_sensor,
            xi_params,
            force_saturation_limit,
            minimum_force_threshold,
            agent_max_steps,
            ExpVecType::NEIGH_INDUCED_RANDOM
        );


        // Run simulator
        simulator.simulate();
        if (i == 0) {
            plot_config(simulator, false, plot_general_name);
        }

        string tmp = "d_p" + to_string(xi_params.d_perf) + "_d_n" + to_string(xi_params.d_none) + "_xi_b" + to_string(xi_params.xi_bar) + "xi_n_b" + to_string(neigh_threshold);
        simulator.save_to_json("uniformity_compare/" + tmp + "/", "run" + to_string(i+1));
    }
}