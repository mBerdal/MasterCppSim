#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "env.h"
#include "helper.h"

#include <Eigen/Dense>
#include <set>
#include <map>
#include <nlohmann/json.hpp>


enum ExpVecType {
    NEIGH_INDUCED,
    NEIGH_INDUCED_RANDOM,
    TOTAL,
};

struct XiParams {
    double d_perf, d_none, xi_bar, agent_neigh_threshold, beacon_neigh_threshold;
};

class Simulator {
public:
    Simulator(double base_dt, double gain_factor, double k_obs, const Env & environment, int num_agents_to_deploy, int num_rays_per_range_sensor,
            const XiParams & xi_params,
            double force_saturation_limit = 4.0, double minimum_force_threshold = 0.01, int agent_max_steps = 100000,
            ExpVecType use_exp_vec_type = ExpVecType::NEIGH_INDUCED);
    
    void simulate();

    inline Env get_environment() const { return environment; }
    inline int get_num_deployed_beacons() const { return 1 + num_agents_to_deploy; }
    inline int get_num_deployed_agents() const { return num_agents_to_deploy; }

    inline std::vector<double> get_beacon_exploration_angles(int beacon_id, ExpVecType exp_vec_type) const { return exploration_angles[beacon_id][exp_vec_type]; }
    inline std::vector<double> get_applied_beacon_exploration_angles(int beacon_id) const { return exploration_angles[beacon_id][use_exp_vec_type]; }

    inline double get_force_saturation_limit() const { return force_saturation_limit; }
    inline double get_minimum_force_threshold() const { return minimum_force_threshold; }

    static const int NUM_STATE_VARIABLES = 5;
    static const int NUM_TRAJ_DATA_POINTS = 14;

    static const int POSITION_X_IDX = 0;
    static const int POSITION_Y_IDX = 1;
    static const int VELOCITY_X_IDX = 2;
    static const int VELOCITY_Y_IDX = 3;
    static const int YAW_IDX = 4;

    static const int F_N_X_IDX = 5;
    static const int F_N_Y_IDX = 6;
    static const int F_E_X_IDX = 7;
    static const int F_E_Y_IDX = 8;
    static const int F_X_IDX = 9;
    static const int F_Y_IDX = 10;
    static const int O_HAT_X_IDX = 11;
    static const int O_HAT_Y_IDX = 12;

    static const int TIMESTAMP_IDX = 13;

    Eigen::MatrixXd get_beacon_traj_data(int agent_id) const { return beacon_traj_data[agent_id]; }
    Eigen::MatrixXd* get_all_traj_data() const { return beacon_traj_data; }

    inline std::vector<std::pair<double, std::vector<int>>> get_agent_neigh_traj(int agent_id) const { return neighbor_set_traj[agent_id - 1]; }
    inline double* get_uniformity_traj() const { return uniformity_traj; }
    inline double get_uniformity_after_deploying_num_agents(int num_agents) const { return uniformity_traj[num_agents]; }

    std::map<int, std::vector<Eigen::Vector2i>> agent_id_to_loop_initiators_map;

    double get_beacon_nominal_weight(int beacon_id) const;

    inline std::string get_saving_string() const { return environment.get_name() + "/" + std::to_string(get_num_deployed_beacons()) + "_beacons"; }
    void save_to_json(std::string data_dir, std::string run_name) const;
    
private:

    static const int TRAJ_DATA_SECTOR_SIZE = 500;
    static constexpr double RANGE_SENSOR_FOV_RAD = 27.0 * M_PI / 180.0;
    static constexpr double RANGE_SENSOR_MAX_RANGE_METERS = 2;

    Eigen::ArrayXd ray_angles_rel_SENSOR;

    enum StepResult {
        NO_NEIGHBORS,
        ZERO_FORCE,
        NO_PROBLEM
    };

    XiParams xi_params;

    Env environment;

    int num_rays_per_range_sensor;
    
    int num_agents_to_deploy;
    int agent_max_steps;

    double force_saturation_limit;
    double minimum_force_threshold;;

    double time;
    double base_dt;

    double k_obs;

    double gain_factor;

    ExpVecType use_exp_vec_type;

    /*
    Trajectory data for each agent. The matrix for beacon with ID i is located
    at beacon_traj_data[i]. Each column stores data for a single timestep.
    */
    Eigen::MatrixXd *beacon_traj_data;


    /*
    Array of maps containing all exploration angles ever set for each beacon.
    Exploration angles for beacon i located at exploration_angles[i].
    Last element of vector exploration_angles[i][exp_vec_type] containes the 
    most recently computed exploration angle of that exp_vec_type.
    */
    std::map<ExpVecType, std::vector<double>>* exploration_angles;

    /*
    Vector of pairs where pairs contain (time, set of neighbor ids).
    Neighbor set traj for agent i located at neighbor_set_traj[i-1].
    */
    std::vector<std::pair<double, std::vector<int>>>* neighbor_set_traj;

    /*
    Array of size num_agents_to_deploy + 1. The value stored at uniformity_traj[i] is the uniformity after
    spawning i agents. 
    */
    double* uniformity_traj;

    Eigen::Vector2d get_total_neigh_force_on_agent(const Eigen::Vector2d & agent_pos, const std::vector<int> & agent_curr_neighs) const;
    Eigen::Vector2d get_single_neigh_force_on_agent(double k_i, const Eigen::Vector2d & agent_pos, const Eigen::Vector2d & beacon_pos, double beacon_exploration_dir, double beacon_xi) const;
    Eigen::Vector2d get_env_force_agent(const Eigen::Vector2d & obstacle_avoidance_vec) const;
    Eigen::Vector2d get_obstacle_avoidance_vector(const Eigen::Vector2d & agent_pos, double agent_yaw) const;
    Eigen::Vector2d get_beacon_pos(int beacon_id) const;

    Eigen::Matrix<double, 4, 2> get_sensed_ranges_and_angles(const Eigen::Vector2d & agent_pos, double agent_yaw) const;

    StepResult do_step(int curr_deploying_agent_id, double* dt_ptr, int step_count);

    std::vector<int> get_neighbors(int id, const Eigen::Vector2d & pos, int max_neigh_id, double neigh_treshold) const;
    std::vector<int> get_beacon_neighbors(int beacon_id, int max_neigh_id) const;
    std::vector<int> get_agent_neighbors(int agent_id, const Eigen::Vector2d & agent_pos) const;

    void set_all_exp_vec_types_for_beacon(int beacon_id, const std::vector<int> &  neighbor_ids, const Eigen::Vector2d & obstacle_avoidance_vec);

    double get_avg_angle_away_from_neighs(int beacon_id, const std::vector<int> &  neighbor_ids) const;

    enum LoopCheckResult {
        NO_LOOP,
        LOOP,
        JITTER
    };
    LoopCheckResult get_loop_check_result(int curr_deploying_agent_id);
    int get_curr_neigh_set_index_of_previous_encounter(int curr_deploying_agent_id) const;

    int neigh_set_repeat_look_back_horizon;
    Eigen::Vector2i most_recent_neigh_set_repeat_indices;

    /*
    NEW STUFF
    */
    CircleSector get_exploration_sector(int curr_deploying_agent_id, const std::vector<int> &  agent_neighbors, const Eigen::Vector2d & obstacle_avoidance_vec) const;
    void compute_beacon_exploration_dir(int beacon_id, int max_neigh_id);

    inline bool did_neigh_set_change(int agent_id, const std::vector<int> & new_neigh_set) { return !vectors_equal(neighbor_set_traj[agent_id - 1].back().second, new_neigh_set); }

    void compute_uniformity(int max_agent_id);

    double get_local_uniformity(int beacon_id, int max_neigh_id) const;
};

#endif