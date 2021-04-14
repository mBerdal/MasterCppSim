#include "sim.h"
#include "helper.h"
#include "range_ray.h"

#include <iostream>

Simulator::Simulator(Env environment, int num_agents_to_deploy, int num_rays_per_range_sensor, Vector2d (*get_force_func)(Vector2d, int, Vector2d, double)) {
    RangeRay::env = environment;

    this->num_agents_to_deploy = num_agents_to_deploy;
    this->get_force_func = get_force_func;
    this->num_rays_per_range_sensor = num_rays_per_range_sensor;
    agent_traj_data = new Matrix<double, NUM_TRAJ_DATA_POINTS, Dynamic>[num_agents_to_deploy];
    beacon_states = new Vector5d[num_agents_to_deploy + 1];
    beacon_states[0] = Vector5d::Zero();
    traj_data_size = 5;

    if (num_rays_per_range_sensor == 1) {
        ray_angles_rel_SENSOR = ArrayXd::Zero(1);
    } else {
        ray_angles_rel_SENSOR = ArrayXd::LinSpaced(num_rays_per_range_sensor, -RANGE_SENSOR_FOV_RAD / 2.0, RANGE_SENSOR_FOV_RAD / 2.0);
    }

    xi_params.d_perf = 1;
    xi_params.d_none = 3;
    xi_params.xi_bar = 2;
    xi_params.neigh_treshold = 0.5;
}

void Simulator::simulate(double dt) {
    time = 0;

    for (int curr_deploying_agent_id = 1; curr_deploying_agent_id <= num_agents_to_deploy; curr_deploying_agent_id++) {
        beacon_states[curr_deploying_agent_id] = Vector5d::Zero();

        agent_traj_data[curr_deploying_agent_id - 1] = Matrix<double, NUM_TRAJ_DATA_POINTS, Dynamic>();
        agent_traj_data[curr_deploying_agent_id - 1].conservativeResize(NUM_TRAJ_DATA_POINTS, traj_data_size);

        int step_count = 0;
        bool land = false;
        while (!land) {
            store_traj_data_points(curr_deploying_agent_id, step_count);
            land = do_step(curr_deploying_agent_id, dt);
            time += dt;
            step_count++;
        }

        // Removing unused data columns
        agent_traj_data[curr_deploying_agent_id - 1] = agent_traj_data[curr_deploying_agent_id - 1].block(0, 0, NUM_TRAJ_DATA_POINTS, step_count);
    }
}

void Simulator::store_traj_data_points(int curr_deploying_agent_id, int step_number) {
    int traj_data_idx = curr_deploying_agent_id - 1;
    if (step_number >= traj_data_size) {
        traj_data_size += traj_data_size;
        agent_traj_data[traj_data_idx].conservativeResize(
            NUM_TRAJ_DATA_POINTS, traj_data_size);
    }
    agent_traj_data[traj_data_idx](TIMESTAMP_IDX, step_number) = time;
    agent_traj_data[traj_data_idx](POSITION_X_IDX, step_number) = beacon_states[curr_deploying_agent_id](POSITION_X_IDX - 1);
    agent_traj_data[traj_data_idx](POSITION_Y_IDX, step_number) = beacon_states[curr_deploying_agent_id](POSITION_Y_IDX - 1);
    agent_traj_data[traj_data_idx](VELOCITY_X_IDX, step_number) = beacon_states[curr_deploying_agent_id](VELOCITY_X_IDX - 1);
    agent_traj_data[traj_data_idx](VELOCITY_Y_IDX, step_number) = beacon_states[curr_deploying_agent_id](VELOCITY_Y_IDX - 1);
    agent_traj_data[traj_data_idx](YAW_IDX, step_number) = beacon_states[curr_deploying_agent_id](YAW_IDX - 1);
}

bool Simulator::do_step(int curr_deploying_agent_id, double dt) {
    Vector2d F_n = get_neigh_force_on_curr_agent(curr_deploying_agent_id);
    Vector2d F_e = get_env_force_on_curr_agent(curr_deploying_agent_id);
    Vector2d F = F_n + F_e;

    Vector5d state_der;
    state_der << F(0), F(1), 0, 0, 0;
    beacon_states[curr_deploying_agent_id] += dt * state_der;
    double F_norm = F.norm();
    return F_norm < 0.01;
}

Vector2d Simulator::get_neigh_force_on_curr_agent(int curr_deploying_agent_id) const {
    Vector2d F = Vector2d::Zero();
    for (int beacon_id = 0; beacon_id < curr_deploying_agent_id; beacon_id++) {
        double dist = (beacon_states[curr_deploying_agent_id] - beacon_states[beacon_id]).norm();
        double xi = get_Xi_from_model(dist, xi_params.d_perf, xi_params.d_none, xi_params.xi_bar);
        if (xi > xi_params.neigh_treshold) {
            F += (*get_force_func)(beacon_states[curr_deploying_agent_id].topRows(2), beacon_id, beacon_states[beacon_id].topRows(2), xi);
        }
    }
    return F;
}

Vector2d Simulator::get_env_force_on_curr_agent(int curr_deploying_agent_id) const {
    Matrix<double, 4, 2> sensed_ranges_and_angles = get_sensed_ranges_and_angles(curr_deploying_agent_id);
    Vector2d F = Vector2d::Zero();
    for (int i = 0; i < 4; i++) {
        if (sensed_ranges_and_angles(i, 0) < RANGE_SENSOR_MAX_RANGE_METERS) {
            Vector2d vec = Rotation2D<double>(sensed_ranges_and_angles(i, 1)).toRotationMatrix() * Vector2d::UnitX();
            F -= vec / pow(sensed_ranges_and_angles(i, 0), 3);
        }
    }
    return F;
}

Matrix<double, 4, 2> Simulator::get_sensed_ranges_and_angles(int curr_deploying_agent_id) const {
    Matrix<double, 4, 2> sensed_ranges_and_angles;
    sensed_ranges_and_angles.col(0) = ((double)RANGE_SENSOR_MAX_RANGE_METERS) * Vector4d::Ones();
    sensed_ranges_and_angles.col(1) = beacon_states[curr_deploying_agent_id](YAW_IDX - 1) * Vector4d::Ones() + Vector4d::LinSpaced(0, (3 / (double)2) * M_PI);

    for (int i = 0; i < 4; i++) {
        double sensor_angle_rel_NED = beacon_states[curr_deploying_agent_id](YAW_IDX - 1) + (double)i * M_PI / 2.0;
        ArrayXd ray_angles_rel_NED = ray_angles_rel_SENSOR + sensor_angle_rel_NED;

        for (int j = 0; j < num_rays_per_range_sensor; j++) {
            double m = RangeRay::sense(beacon_states[curr_deploying_agent_id].topRows(2), ray_angles_rel_NED(j), RANGE_SENSOR_MAX_RANGE_METERS);
            if (m < sensed_ranges_and_angles(i, 0)) {
                sensed_ranges_and_angles(i, 0) = m;
                sensed_ranges_and_angles(i, 1) = ray_angles_rel_NED(j);
            }
        }
    }
    return sensed_ranges_and_angles;
}
