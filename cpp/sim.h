#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "env.h"
#include "helper.h"

#include "Eigen/Dense"
#include <set>
#include <map>

enum ExpVecType {
    NEIGH_INDUCED,
    NEIGH_INDUCED_RANDOM,
    TOTAL,
};

class Simulator {
public:
    Simulator(double base_dt, double k_obs, Env environment, int num_agents_to_deploy, int num_rays_per_range_sensor,
            Eigen::Vector2d (*get_force_func)(Eigen::Vector2d, int, Eigen::Vector2d, double, double),
            double force_saturation_limit = 4.0, double minimum_force_threshold = 0.01, int agent_max_steps = 100000,
            ExpVecType use_exp_vec_type = ExpVecType::NEIGH_INDUCED);
    
    void simulate();

    inline Env get_environment() const { return environment; }
    inline int get_num_deployed_beacons() const { return 1 + num_agents_to_deploy; }
    inline int get_num_deployed_agents() const { return num_agents_to_deploy; }

    inline std::vector<double> get_beacon_exploration_angles(int beacon_id, ExpVecType exp_vec_type) const { return exploration_angles[beacon_id][exp_vec_type]; }
    inline std::vector<double> get_applied_beacon_exploration_angles(int beacon_id) const { return exploration_angles[beacon_id][use_exp_vec_type]; }

    inline double get_force_saturation_limit() { return force_saturation_limit; }
    inline double get_minimum_force_threshold() { return minimum_force_threshold; }

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

    std::map<int, std::vector<std::pair<int, int>>> agent_id_neigh_traj_idx_of_loop_start_end_map;
    
private:

    static const int TRAJ_DATA_SECTOR_SIZE = 500;
    static constexpr double RANGE_SENSOR_FOV_RAD = 27.0 * M_PI / 180.0;
    static constexpr double RANGE_SENSOR_MAX_RANGE_METERS = 2;

    int num_rays_per_range_sensor;
    Eigen::ArrayXd ray_angles_rel_SENSOR;

    enum StepResult {
        NO_NEIGHBORS,
        ZERO_FORCE,
        NO_PROBLEM
    };

    struct XiParams {
        double d_perf;
        double d_none;
        double xi_bar;
        double neigh_treshold;
    };
    XiParams xi_params;

    double force_saturation_limit;
    double minimum_force_threshold;;


    Env environment;
    int num_agents_to_deploy;
    int agent_max_steps;
    double time;
    double base_dt;
    double k_obs;
    ExpVecType use_exp_vec_type;

    Eigen::Vector2d (*get_force_func)(Eigen::Vector2d, int, Eigen::Vector2d, double, double);

    /*
    Trajectory data for each agent. The matrix for beacon with ID i is located
    at beacon_traj_data[i]. Each column stores data for a single timestep.
    */
    Eigen::MatrixXd *beacon_traj_data;

    std::map<ExpVecType, std::vector<double>>* exploration_angles;

    /*
    Each vector contains tuples of (time, set of neighbor ids)
    */
    std::vector<std::pair<double, std::vector<int>>>* neighbor_set_traj;

    Eigen::Vector2d get_neigh_force_on_agent(Eigen::Vector2d agent_pos, std::vector<int> agent_curr_neighs) const;
    Eigen::Vector2d get_env_force_agent(Eigen::Vector2d obstacle_avoidance_vec) const;
    Eigen::Vector2d get_obstacle_avoidance_vector(Eigen::Vector2d agent_pos, double agent_yaw) const;

    Eigen::Matrix<double, 4, 2> get_sensed_ranges_and_angles(Eigen::Vector2d agent_pos, double agent_yaw) const;

    StepResult do_step(int curr_deploying_agent_id, double* dt_ptr, int step_count);

    std::vector<int> get_beacon_neighbors(int beacon_id, Eigen::Vector2d beacon_pos, int max_neigh_id) const;
    void set_all_exp_vec_types_for_beacon(int beacon_id, std::vector<int> neighbor_ids, Eigen::Vector2d obstacle_avoidance_vec);

    double get_avg_angle_away_from_neighs(int beacon_id, std::vector<int> neighbor_ids) const;
    double get_wall_adjusted_angle(double nominal_angle, Eigen::Vector2d obstacle_avoidance_vec) const;
    
    bool get_is_looping(int curr_deploying_agent_id);
    int get_curr_neigh_set_index_of_encounter(int curr_deploying_agent_id) const;
    int neighs_encountered_before_idx;
    int neigh_look_back_horizon;

    /*
    NEW STUFF
    */
    CircleSector get_exploration_sector(int curr_deploying_agent_id, std::vector<int> agent_neighbors, Eigen::Vector2d obstacle_avoidance_vec) const;
};

#endif