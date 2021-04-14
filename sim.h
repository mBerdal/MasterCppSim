#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "env.h"

#include "Eigen/Dense"
using namespace Eigen;

#define Vector5d Matrix<double, 5, 1>

class Simulator {
private:

    struct XiParams
    {
        double d_perf;
        double d_none;
        double xi_bar;
        double neigh_treshold;
    };

    XiParams xi_params;
    
    static const int NUM_TRAJ_DATA_POINTS = 6;

    static const int TIMESTAMP_IDX = 0;
    static const int POSITION_X_IDX = 1;
    static const int POSITION_Y_IDX = 2;
    static const int VELOCITY_X_IDX = 3;
    static const int VELOCITY_Y_IDX = 4;
    static const int YAW_IDX = 5;

    static constexpr double RANGE_SENSOR_FOV_RAD = 27.0 * M_PI / 180.0;
    static constexpr double RANGE_SENSOR_MAX_RANGE_METERS = 4;
    int num_rays_per_range_sensor;
    ArrayXd ray_angles_rel_SENSOR;

    int traj_data_size;

    static constexpr double neigh_Xi_threshold = 0.5;

    Vector5d *beacon_states;
    int num_agents_to_deploy;
    int curr_deploying_agent_id;
    double time;

    Vector2d (*get_force_func)(Vector2d, int, Vector2d, double);

    /*
    Trajectory data for each agent. The matrix for agent with ID i is located
    at agent_traj_data[i].
    
    agent_traj_data[i](0, j): Timestamp number j.
    agent_traj_data[i](1, j): x-position of agent i at time agent_traj_data[i](0, j).
    agent_traj_data[i](2, j): y-position of agent i at time agent_traj_data[i](0, j).
    agent_traj_data[i](3, j): x-velocity of agent i at time agent_traj_data[i](0, j).
    agent_traj_data[i](4, j): y-velocity of agent i at time agent_traj_data[i](0, j).
    */
    Matrix<double, 6, Dynamic> *agent_traj_data;

    void expand_traj_data_length(int traj_data_idx);
    void store_traj_data_points(int curr_deploying_agent_id, int step_number);
    Vector2d get_neigh_force_on_curr_agent(int curr_deploying_agent_id) const;
    Vector2d get_env_force_on_curr_agent(int curr_deploying_agent_id) const;
    Matrix<double, 4, 2> get_sensed_ranges_and_angles(int curr_deploying_agent_id) const;
    bool do_step(int curr_deploying_agent_id, double dt);

public:
    Simulator(Env environment, int num_agents_to_deploy, int num_rays_per_range_sensor, Vector2d (*get_force_func)(Vector2d, int, Vector2d, double));
    void simulate(double dt);

    Matrix<double, 6, Dynamic> *get_agent_traj_data() const { return agent_traj_data; }
};

#endif