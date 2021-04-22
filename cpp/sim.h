#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "env.h"

#include "Eigen/Dense"
using namespace Eigen;

#include <tuple>
#include <set>
#include <map>
using namespace std;

#define Vector5d Matrix<double, 5, 1>

enum ExpVecType {
    NEIGH_INDUCED,
    NEIGH_INDUCED_RANDOM,
    OBS_AVOIDANCE,
    TOTAL,
};

class Simulator {
public:
    Simulator(double base_dt, double k_obs, Env environment, int num_agents_to_deploy, int num_rays_per_range_sensor,
            Vector2d (*get_force_func)(Vector2d, int, Vector2d, Vector2d, double),
            double force_saturation_limit = 4.0, double minimum_force_threshold = 0.01, int agent_max_steps = 100000,
            ExpVecType use_exp_vec_type = ExpVecType::NEIGH_INDUCED);
    
    void simulate();

    inline Env get_environment() const { return environment; }
    inline int get_num_deployed_beacons() const { return 1 + num_agents_to_deploy; }
    inline int get_num_deployed_agents() const { return num_agents_to_deploy; }

    inline vector<Vector2d> get_beacon_exploration_dirs(int beacon_id, ExpVecType exp_vec_type) const { return exploration_vectors[beacon_id][exp_vec_type]; }
    inline vector<Vector2d> get_applied_beacon_exploration_dirs(int beacon_id) const { return exploration_vectors[beacon_id][use_exp_vec_type]; }

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


    MatrixXd get_beacon_traj_data(int agent_id) const { return beacon_traj_data[agent_id]; }
    MatrixXd* get_all_traj_data() const { return beacon_traj_data; }

    inline vector<pair<double, vector<int>>> get_agent_neigh_traj(int agent_id) const { return neighbor_set_traj[agent_id - 1]; }

    map<int, vector<pair<int, int>>> agent_id_neigh_traj_idx_of_loop_start_end_map;
    
private:

    static const int TRAJ_DATA_SECTOR_SIZE = 500;
    static constexpr double RANGE_SENSOR_FOV_RAD = 27.0 * M_PI / 180.0;
    static constexpr double RANGE_SENSOR_MAX_RANGE_METERS = 2;

    int num_rays_per_range_sensor;
    ArrayXd ray_angles_rel_SENSOR;

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

    Vector2d (*get_force_func)(Vector2d, int, Vector2d, Vector2d, double);

    /*
    Trajectory data for each agent. The matrix for beacon with ID i is located
    at beacon_traj_data[i]. Each column stores data for a single timestep.
    */
    MatrixXd *beacon_traj_data;

    map<ExpVecType, vector<Vector2d>>* exploration_vectors;

    /*
    Each vector contains tuples of (time, set of neighbor ids)
    */
    vector<pair<double, vector<int>>>* neighbor_set_traj;

    Vector2d get_neigh_force_on_agent(Vector2d agent_pos, vector<int> agent_curr_neighs) const;
    Vector2d get_env_force_agent(Vector2d obstacle_avoidance_vec) const;
    Vector2d get_obstacle_avoidance_vector(Vector2d agent_pos, double agent_yaw) const;

    Matrix<double, 4, 2> get_sensed_ranges_and_angles(Vector2d agent_pos, double agent_yaw) const;

    StepResult do_step(int curr_deploying_agent_id, double* dt_ptr, int step_count);

    vector<int> get_beacon_neighbors(int beacon_id, Vector2d beacon_pos, int max_neigh_id) const;
    double get_neigh_induced_exploration_angle_for_beacon(int beacon_id, vector<int> neighbor_ids) const;
    void set_all_exp_vec_types_for_beacon(int beacon_id, vector<int> neighbor_ids, Vector2d obstacle_avoidance_vec);

    bool get_is_looping(int curr_deploying_agent_id, vector<int> curr_deploying_agent_curr_neighs);
    int get_index_of_encounter(int curr_deploying_agent_id, vector<int> curr_deploying_agent_curr_neighs) const;
    int neighs_encountered_before_idx;
    int neigh_look_back_horizon;
};

#endif