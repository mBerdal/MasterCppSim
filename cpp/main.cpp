#include "env.h"
#include "sim.h"
#include "plotter.h"


#include <Eigen/Dense>
namespace eig = Eigen;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <list>
#include <fstream>
#include <iostream>
using namespace std;

#define DATA_DIR string("../../data/")

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
    Env environment = Env::ten_by_ten;

    // Parameters
    int num_rays_per_range_sensor = 1;
    int agent_max_steps = 100000;

    double base_dt = 10e-3;
    double gain_factor = 1.5;
    double k_obs = 0.1;
    double force_saturation_limit = 4.0;
    double minimum_force_threshold = 0.01;

    XiParams xi_params;
    xi_params.d_perf = 2; //3;
    xi_params.d_none = 4.2248985876; //6;
    xi_params.xi_bar = 20; //20;
    xi_params.neigh_treshold = 0.5;

    // Data storage
    json data_storage;


    int num_runs_per_swarm_size = 10;

    // Initializing simulator with parameters
    for (int num_agents_to_deploy = 50; num_agents_to_deploy < 60; num_agents_to_deploy++) {
        vector<double> uniformities = vector<double>(num_runs_per_swarm_size, 0);
        for (int i = 0; i < num_runs_per_swarm_size; i++) {
            Simulator simulator(
                base_dt,
                gain_factor,
                k_obs,
                environment,
                num_agents_to_deploy,
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
            //plot_config(simulator);
            //plot_uniformity_traj(simulator);
            uniformities[i] = simulator.get_uniformity_after_deploying_num_agents(num_agents_to_deploy);
        }
        data_storage[to_string(num_agents_to_deploy + 1)] = uniformities;
        cout << "DONE WITH " << num_agents_to_deploy << "\n";

        // Plotting result of simulation
        // string general_name = "sector_random_hallway__xploration";
        // plot_uniformity_traj(simulator, general_name);
        // plot_config(simulator, general_name);
        // for (const pair<int, vector<eig::Vector2i>> & agent_id_loop_initiators_pair : simulator.agent_id_to_loop_initiators_map) {
        //     plot_single_beacon_traj(simulator, agent_id_loop_initiators_pair.first, true, true, general_name + "looping");
        //     plot_agent_neigh_traj(simulator, agent_id_loop_initiators_pair.first, general_name + "looping");
        // }
    }
    
    std::ofstream o(DATA_DIR + "uniformity_test_v3.json");
    o << std::setw(4) << data_storage << std::endl;


    return 0;
}