#include "sim.h"
#include "helper.h"
#include "range_ray.h"

#include <iostream>


Simulator::Simulator(double base_dt, double k_obs, Env environment, int num_agents_to_deploy, int num_rays_per_range_sensor,
                    Vector2d (*get_force_func)(Vector2d, int, Vector2d, Vector2d, double),
                    double force_saturation_limit, double minimum_force_threshold, int agent_max_steps,
                    ExpVecType use_exp_vec_type) {

    this->base_dt = base_dt;
    this->k_obs = k_obs;
    RangeRay::env = environment;
    this->environment = environment;
    this->num_agents_to_deploy = num_agents_to_deploy;
    this->get_force_func = get_force_func;
    this->force_saturation_limit = force_saturation_limit;
    this->minimum_force_threshold = minimum_force_threshold;
    this->agent_max_steps = agent_max_steps;
    this->use_exp_vec_type = use_exp_vec_type;
    
    beacon_traj_data = new MatrixXd[1 + num_agents_to_deploy];
    beacon_traj_data[0] = Matrix<double, NUM_TRAJ_DATA_POINTS, 1>::Zero();

    exploration_vectors = new map<ExpVecType, Vector2d>[1 + num_agents_to_deploy];
    set_all_exp_vec_types_for_beacon(0, {});

    this->num_rays_per_range_sensor = num_rays_per_range_sensor;
    if (num_rays_per_range_sensor == 1) {
        ray_angles_rel_SENSOR = ArrayXd::Zero(1);
    } else {
        ray_angles_rel_SENSOR = ArrayXd::LinSpaced(num_rays_per_range_sensor, -RANGE_SENSOR_FOV_RAD / 2.0, RANGE_SENSOR_FOV_RAD / 2.0);
    }

    xi_params.d_perf = 1;
    xi_params.d_none = 3;
    xi_params.xi_bar = 3;
    xi_params.neigh_treshold = 0.5;

    RandomNumberGenerator::seed();
}

void Simulator::simulate() {
    time = 0;

    for (int curr_deploying_agent_id = 1; curr_deploying_agent_id <= num_agents_to_deploy; curr_deploying_agent_id++) {
        
        beacon_traj_data[curr_deploying_agent_id] = MatrixXd();
        beacon_traj_data[curr_deploying_agent_id].conservativeResize(NUM_TRAJ_DATA_POINTS, TRAJ_DATA_SECTOR_SIZE);

        beacon_traj_data[curr_deploying_agent_id].topLeftCorner(NUM_STATE_VARIABLES, 1) = Vector5d::Zero();
        beacon_traj_data[curr_deploying_agent_id].block(F_N_X_IDX, 0, 6, 1) = Vector<double, 6>::Zero();
        beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, 0) = time;

        int step_count = 0;
        double* dt_ptr;
        StepResult step_result = NO_PROBLEM;
        while (step_result == NO_PROBLEM && step_count < agent_max_steps) {
            step_result = do_step(curr_deploying_agent_id, dt_ptr, step_count);
            time += *dt_ptr;
            step_count++;
        }

        cout << "Agent " << curr_deploying_agent_id << " landing after " << step_count << " steps\n";

        /* Removing unused data columns
        
        If deployment was haulted due to lack of neighbors before performing step,
        we pretend that the step causing the drone to loose contact never happened.
        If deployment was haulted due to a lack of force, we are happy and include
        the step taken before haulting due to a lack of force.
        */
        if (step_result == NO_NEIGHBORS) {
            step_count--;
            time -= (*dt_ptr);
            cout << "Agent " << curr_deploying_agent_id << " landed due to a lack of neighbors\n";
        }

        MatrixXd valid_traj_data = beacon_traj_data[curr_deploying_agent_id].leftCols(step_count);
        beacon_traj_data[curr_deploying_agent_id] = valid_traj_data;

        set<int> agent_neighbors_at_landing = get_agent_neighbors(
            curr_deploying_agent_id,
            beacon_traj_data[curr_deploying_agent_id].topRightCorner(2, 1)
        );

        set_all_exp_vec_types_for_beacon(curr_deploying_agent_id, agent_neighbors_at_landing);

        std::cout << "Agent " << curr_deploying_agent_id << " landed at\n" << beacon_traj_data[curr_deploying_agent_id].topRightCorner(2, 1) << "\n";
    }
}

Simulator::StepResult Simulator::do_step(int curr_deploying_agent_id, double* dt_ptr, int step_count) {
    if (step_count + 1 >= beacon_traj_data[curr_deploying_agent_id].cols()) {
        beacon_traj_data[curr_deploying_agent_id].conservativeResize(
            NUM_TRAJ_DATA_POINTS,
            beacon_traj_data[curr_deploying_agent_id].cols() + TRAJ_DATA_SECTOR_SIZE
        );
    }

    Vector5d curr_deploying_agent_state = beacon_traj_data[curr_deploying_agent_id].block(0, step_count, NUM_STATE_VARIABLES, 1);
    Vector2d curr_deploying_agent_pos = curr_deploying_agent_state.head(2);
    double curr_deploying_agent_yaw = curr_deploying_agent_state(YAW_IDX);

    set<int> curr_deploying_agent_curr_neighs = get_agent_neighbors(
        curr_deploying_agent_id,
        curr_deploying_agent_pos
    );
    if (curr_deploying_agent_curr_neighs.size() == 0) {
        return NO_NEIGHBORS;
    }

    Vector2d F_n = get_neigh_force_on_agent(
        curr_deploying_agent_pos,
        curr_deploying_agent_curr_neighs
    );

    Vector2d F_e = get_env_force_agent(
        curr_deploying_agent_pos,
        curr_deploying_agent_yaw
    );

    Vector2d F_nominal = F_n + F_e;
    Vector2d F = clamp_vec(F_nominal, force_saturation_limit);
    if (F.norm() < minimum_force_threshold) {
        return ZERO_FORCE;
    }
    
    Vector5d state_der;
    state_der << F(0), F(1), 0, 0, 0;

    *dt_ptr = base_dt;
    if (F_e.norm() > 50) {
        *dt_ptr = base_dt / 100.0;
    }
    else if (F_e.norm() > 10) {
        *dt_ptr = base_dt / 10.0;
    }

    beacon_traj_data[curr_deploying_agent_id].block(0, step_count + 1, NUM_STATE_VARIABLES, 1) = \
    curr_deploying_agent_state + (*dt_ptr) * state_der;

    beacon_traj_data[curr_deploying_agent_id].block(F_N_X_IDX, step_count + 1, 2, 1) = F_n;
    beacon_traj_data[curr_deploying_agent_id].block(F_E_X_IDX, step_count + 1, 2, 1) = F_e;
    beacon_traj_data[curr_deploying_agent_id].block(F_X_IDX, step_count + 1, 2, 1) = F_nominal;
    beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, step_count + 1) = time;

    return NO_PROBLEM;
}

Vector2d Simulator::get_neigh_force_on_agent(Vector2d agent_pos, set<int> agent_curr_neighs) const {
    Vector2d F = Vector2d::Zero();

    for (int beacon_id : agent_curr_neighs) {
        Vector2d other_beacon_pos = beacon_traj_data[beacon_id].topRightCorner(2, 1);

        double dist = (agent_pos - other_beacon_pos).norm();
        double xi = get_Xi_from_model(dist, xi_params.d_perf, xi_params.d_none, xi_params.xi_bar);

        F += (*get_force_func)(agent_pos, beacon_id, other_beacon_pos, exploration_vectors[beacon_id][use_exp_vec_type], xi);
    }
    return F;
}

Vector2d Simulator::get_env_force_agent(Vector2d agent_pos, double agent_yaw) const {
    Vector2d o_hat = get_obstacle_avoidance_vector(agent_pos, agent_yaw);
    return k_obs * (1 / (RANGE_SENSOR_MAX_RANGE_METERS - o_hat.norm())) * o_hat;
}

Vector2d Simulator::get_obstacle_avoidance_vector(Vector2d agent_pos, double agent_yaw) const {
    Matrix<double, 4, 2> sensed_ranges_and_angles = get_sensed_ranges_and_angles(
        agent_pos,
        agent_yaw
    );
    
    Vector2d o = Vector2d::Zero();
    for (int i = 0; i < 4; i++) {
        o += sensed_ranges_and_angles(i, 0) * Rotation2D<double>(sensed_ranges_and_angles(i, 1)).toRotationMatrix() * Vector2d::UnitX();
    }

    double o_norm = o.norm();
    return o_norm <= RANGE_SENSOR_MAX_RANGE_METERS ? o : (RANGE_SENSOR_MAX_RANGE_METERS / o_norm)*o;
}

Matrix<double, 4, 2> Simulator::get_sensed_ranges_and_angles(Vector2d agent_pos, double agent_yaw) const {

    Matrix<double, 4, 2> sensed_ranges_and_angles;
    sensed_ranges_and_angles.col(0) = ((double)RANGE_SENSOR_MAX_RANGE_METERS) * Vector4d::Ones();
    sensed_ranges_and_angles.col(1) = agent_yaw * Vector4d::Ones() + Vector4d::LinSpaced(0, (3 / 2.0) * M_PI);

    for (int i = 0; i < 4; i++) {
        double sensor_angle_rel_NED = agent_yaw + (double)i * M_PI / 2.0;
        ArrayXd ray_angles_rel_NED = ray_angles_rel_SENSOR + sensor_angle_rel_NED;

        for (int j = 0; j < num_rays_per_range_sensor; j++) {
            double m = RangeRay::sense(agent_pos, ray_angles_rel_NED(j), RANGE_SENSOR_MAX_RANGE_METERS);
            if (m < sensed_ranges_and_angles(i, 0)) {
                sensed_ranges_and_angles(i, 0) = m;
                sensed_ranges_and_angles(i, 1) = ray_angles_rel_NED(j);
            }
        }
    }
    return sensed_ranges_and_angles;
}

set<int> Simulator::get_agent_neighbors(int agent_id, Vector2d agent_pos) const {
    set<int> neighs;
    for (int other_beacon_id=0; other_beacon_id < agent_id; other_beacon_id++) {
        Vector2d other_beacon_pos = beacon_traj_data[other_beacon_id].topRightCorner(2, 1);
        double dist = (agent_pos - other_beacon_pos).norm();
        if (get_Xi_from_model(dist, xi_params.d_perf, xi_params.d_none, xi_params.xi_bar) > xi_params.neigh_treshold) {
            neighs.insert(other_beacon_id);
        }
    }
    return neighs;
}

void Simulator::set_all_exp_vec_types_for_beacon(int beacon_id, set<int> neighbors_at_landing_ids){
    exploration_vectors[beacon_id][ExpVecType::NOMINAL] = get_nominal_exploration_vector_for_beacon(
            beacon_id, neighbors_at_landing_ids
    );
    Matrix2d R = Rotation2Dd((M_PI / 4.0) * RandomNumberGenerator::get_between(-1, 1)).toRotationMatrix();
    exploration_vectors[beacon_id][ExpVecType::NOMINAL_RAND] = R*exploration_vectors[beacon_id][ExpVecType::NOMINAL];
}

Vector2d Simulator::get_nominal_exploration_vector_for_beacon(int beacon_id, set<int> neighbors_at_landing_ids) const {
    if (neighbors_at_landing_ids.size() == 0) {
        cout << "Beacon " << beacon_id << " has no neighbors. Using unit x vector as exploration vector.\n";
        return Vector2d::UnitX();
    }

    double sum_of_weights = 0;
    for (int neigh_id : neighbors_at_landing_ids) {
        sum_of_weights += neigh_id + 1;
    }
    Vector2d weighted_avg_vec_to_neighs = Vector2d::Zero();
    for (int neigh_id : neighbors_at_landing_ids) {
        weighted_avg_vec_to_neighs += ((neigh_id + 1) / sum_of_weights)*(
            beacon_traj_data[beacon_id].topRightCorner(2, 1) - beacon_traj_data[neigh_id].topRightCorner(2, 1)
        );
    }
    return weighted_avg_vec_to_neighs.normalized();
}

