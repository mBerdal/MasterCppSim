#include "sim.h"
#include "helper.h"
#include "range_ray.h"

#include <iostream>

#define TWO_PI 2 * M_PI
//#define double ang_diff = remainder(o_hat_angle - theta_nom, TWO_PI)

double clamp_pm_pi(double ang) {
    return remainder(ang, TWO_PI);
}

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

    neighbor_set_traj = new vector<tuple<double, vector<int>>>[num_agents_to_deploy];

    exploration_vectors = new map<ExpVecType, Vector2d>[1 + num_agents_to_deploy];
    set_all_exp_vec_types_for_beacon(0, {}, Vector2d::Zero());


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

    looping_agents = set<int>();
}

void Simulator::simulate() {
    time = 0;
    double dt;
    
    for (int curr_deploying_agent_id = 1; curr_deploying_agent_id <= num_agents_to_deploy; curr_deploying_agent_id++) {
        
        beacon_traj_data[curr_deploying_agent_id] = MatrixXd();
        beacon_traj_data[curr_deploying_agent_id].conservativeResize(NUM_TRAJ_DATA_POINTS, TRAJ_DATA_SECTOR_SIZE);

        beacon_traj_data[curr_deploying_agent_id].topLeftCorner(NUM_STATE_VARIABLES, 1) = Vector5d::Zero();
        beacon_traj_data[curr_deploying_agent_id].block(F_N_X_IDX, 0, 6, 1) = Vector<double, 6>::Zero();
        beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, 0) = time;

        int step_count = 0;
        neighs_encountered_before_idx = -1;
        StepResult step_result = NO_PROBLEM;
        while (step_result == NO_PROBLEM && step_count < agent_max_steps) {
            dt = base_dt;
            step_result = do_step(curr_deploying_agent_id, &dt, step_count);
            time += dt;
            step_count++;
        }

        /* Removing unused data columns
        
        If deployment was haulted due to lack of neighbors before performing step,
        we pretend that the step causing the drone to loose contact never happened.
        If deployment was haulted due to a lack of force, we are happy and include
        the step taken before haulting due to a lack of force.
        */
        if (step_result == NO_NEIGHBORS) {
            step_count--;
            time -= dt;
            cout << "Agent " << curr_deploying_agent_id << " landed due to a lack of neighbors\n";
        }

        MatrixXd valid_traj_data = beacon_traj_data[curr_deploying_agent_id].leftCols(step_count);
        beacon_traj_data[curr_deploying_agent_id] = valid_traj_data;

        vector<int> agent_neighbors_at_landing = get_agent_neighbors(
            curr_deploying_agent_id,
            beacon_traj_data[curr_deploying_agent_id].topRightCorner(2, 1)
        );

        Vector2d o_hat_at_landing = beacon_traj_data[curr_deploying_agent_id].block(
            O_HAT_X_IDX,
            beacon_traj_data[curr_deploying_agent_id].cols()-1,
            2,
            1
        );
        
        set_all_exp_vec_types_for_beacon(curr_deploying_agent_id, agent_neighbors_at_landing, o_hat_at_landing);

        cout << "Agent " << curr_deploying_agent_id << " landed after " << step_count << " steps\n";
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

    vector<int> curr_deploying_agent_curr_neighs = get_agent_neighbors(
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

    Vector2d o_hat = get_obstacle_avoidance_vector(curr_deploying_agent_pos, curr_deploying_agent_yaw);
    Vector2d F_e = get_env_force_agent(o_hat);

    Vector2d F_nominal = F_n + F_e;
    Vector2d F = clamp_vec(F_nominal, force_saturation_limit);
    if (F.norm() < minimum_force_threshold) {
        return ZERO_FORCE;
    }
    
    Vector5d state_der;
    state_der << F(0), F(1), 0, 0, 0;
    /**
     * TODO: Fix dynamic step size
     **/
    double F_e_norm = F_e.norm();
    if (F_e_norm > 50) {
        *dt_ptr = base_dt / 100.0;
    }
    else if (F_e_norm > 2) {
        *dt_ptr = base_dt / 10.0;
    }
    if (step_count > agent_max_steps / 2) {
        *dt_ptr = base_dt / 1000.0;
    }

    beacon_traj_data[curr_deploying_agent_id].block(0, step_count + 1, NUM_STATE_VARIABLES, 1) = \
    curr_deploying_agent_state + (*dt_ptr) * state_der;

    beacon_traj_data[curr_deploying_agent_id].block(F_N_X_IDX, step_count + 1, 2, 1) = F_n;
    beacon_traj_data[curr_deploying_agent_id].block(F_E_X_IDX, step_count + 1, 2, 1) = F_e;
    beacon_traj_data[curr_deploying_agent_id].block(F_X_IDX, step_count + 1, 2, 1) = F_nominal;
    beacon_traj_data[curr_deploying_agent_id].block(O_HAT_X_IDX, step_count + 1, 2, 1) = o_hat;
    beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, step_count + 1) = time;
    
    if (step_count == 0 || get<1>(neighbor_set_traj[curr_deploying_agent_id - 1].back()) != curr_deploying_agent_curr_neighs) {
        /*
        Neighbor set changed (or first step)
        */
        if (is_loop_detected(curr_deploying_agent_id, curr_deploying_agent_curr_neighs)){
            looping_agents.insert(curr_deploying_agent_id);
        }
        neighbor_set_traj[curr_deploying_agent_id - 1].push_back(
            tuple<double, vector<int>>(
                beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, step_count),
                curr_deploying_agent_curr_neighs
            )
        );
    }
    return NO_PROBLEM;
}

Vector2d Simulator::get_neigh_force_on_agent(Vector2d agent_pos, vector<int> agent_curr_neighs) const {
    Vector2d F = Vector2d::Zero();

    for (int beacon_id : agent_curr_neighs) {
        Vector2d other_beacon_pos = beacon_traj_data[beacon_id].topRightCorner(2, 1);

        double dist = (agent_pos - other_beacon_pos).norm();
        double xi = get_Xi_from_model(dist, xi_params.d_perf, xi_params.d_none, xi_params.xi_bar);

        F += (*get_force_func)(agent_pos, beacon_id, other_beacon_pos, exploration_vectors[beacon_id][use_exp_vec_type], xi);
    }
    return F;
}

Vector2d Simulator::get_env_force_agent(Vector2d obstacle_avoidance_vec) const {
    return k_obs * (1 / (RANGE_SENSOR_MAX_RANGE_METERS - obstacle_avoidance_vec.norm())) * obstacle_avoidance_vec;
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

vector<int> Simulator::get_agent_neighbors(int agent_id, Vector2d agent_pos) const {
    vector<int> neighs;
    for (int other_beacon_id=0; other_beacon_id < agent_id; other_beacon_id++) {
        Vector2d other_beacon_pos = beacon_traj_data[other_beacon_id].topRightCorner(2, 1);
        double dist = (agent_pos - other_beacon_pos).norm();
        if (get_Xi_from_model(dist, xi_params.d_perf, xi_params.d_none, xi_params.xi_bar) > xi_params.neigh_treshold) {
            neighs.push_back(other_beacon_id);
        }
    }
    return neighs;
}

void Simulator::set_all_exp_vec_types_for_beacon(int beacon_id, vector<int> neighbors_at_landing_ids, Vector2d obstacle_avoidance_vec){
    double theta_neighs = get_neigh_induced_exploration_angle_for_beacon(beacon_id, neighbors_at_landing_ids);
    exploration_vectors[beacon_id][ExpVecType::NEIGH_INDUCED] = Rotation2Dd(theta_neighs).toRotationMatrix()*Vector2d::UnitX();

    double theta_nom = clamp_pm_pi(M_PI_4 * RandomNumberGenerator::get_between(-1, 1) + theta_neighs);
    exploration_vectors[beacon_id][ExpVecType::NEIGH_INDUCED_RANDOM] = Rotation2Dd(theta_nom).toRotationMatrix()*Vector2d::UnitX();

    double o_hat_angle = atan2(obstacle_avoidance_vec(1), obstacle_avoidance_vec(0));
    double ang_diff = clamp_pm_pi(o_hat_angle - theta_nom);
    double theta_obs = theta_nom;
    if (ang_diff < -M_PI_2) {
        theta_obs = o_hat_angle + M_PI_2;
    }
    else if (ang_diff > M_PI_2) {
        theta_obs = o_hat_angle - M_PI_2;
    }

    double o_hat_norm = obstacle_avoidance_vec.norm();
    if (o_hat_norm > 10e-7) {
        exploration_vectors[beacon_id][ExpVecType::OBS_AVOIDANCE] = Rotation2Dd(theta_obs).toRotationMatrix()*Vector2d::UnitX();
    }
    else {
        exploration_vectors[beacon_id][ExpVecType::OBS_AVOIDANCE] = Vector2d::Zero();
    }

    double w_ang = o_hat_norm / RANGE_SENSOR_MAX_RANGE_METERS;
    double c = (1 - w_ang)*cos(theta_nom) + w_ang*cos(theta_obs);
    double s = (1 - w_ang)*sin(theta_nom) + w_ang*sin(theta_obs);

    exploration_vectors[beacon_id][ExpVecType::TOTAL] = Rotation2Dd(atan2(s, c)).toRotationMatrix()*Vector2d::UnitX();
}

double Simulator::get_neigh_induced_exploration_angle_for_beacon(int beacon_id, vector<int> neighbors_at_landing_ids) const {
    if (neighbors_at_landing_ids.size() == 0) {
        cout << "Beacon " << beacon_id << " has no neighbors. Using zero as exploration angle.\n";
        return 0;
    }

    double sum_of_weights = 0;
    for (int neigh_id : neighbors_at_landing_ids) {
        sum_of_weights += neigh_id + 1;
    }
    Vector2d weighted_avg_vec_to_neighs = Vector2d::Zero();
    for (int neigh_id : neighbors_at_landing_ids) {
        weighted_avg_vec_to_neighs += ((neigh_id + 1) / sum_of_weights)*(
            beacon_traj_data[beacon_id].topRightCorner(2, 1) - beacon_traj_data[neigh_id].topRightCorner(2, 1)
        ).normalized();
    }
    return atan2(weighted_avg_vec_to_neighs(1), weighted_avg_vec_to_neighs(0));
}

int Simulator::get_index_of_encounter(int curr_deploying_agent_id, vector<int> curr_deploying_agent_curr_neighs) const {
    int num_curr_neighs = curr_deploying_agent_curr_neighs.size();
    for (int i = 0; i < neighbor_set_traj[curr_deploying_agent_id - 1].size(); i++) {
        vector<int> recorded_neigh_set = get<1>(neighbor_set_traj[curr_deploying_agent_id - 1][i]);

        bool all_equal = false;
        if (recorded_neigh_set.size() == num_curr_neighs) {
            all_equal = true;
            for (int j=0; j < num_curr_neighs; j++) {
                if (curr_deploying_agent_curr_neighs[j] != recorded_neigh_set[j]) {
                    all_equal = false;
                    break;
                }
            }
        }
        if (all_equal) {
            return i;
        }
    }
    return -1;
}

bool Simulator::is_loop_detected(int curr_deploying_agent_id, vector<int> curr_deploying_agent_curr_neighs) {
    if (neighs_encountered_before_idx == -1) {
        /*
        No previous neighbor set ever encountered before. Check if current neighbor set has been encountered before.
        If it has, set neighs_encountered_before_idx to that index
        */
        neighs_encountered_before_idx = get_index_of_encounter(curr_deploying_agent_id, curr_deploying_agent_curr_neighs);
    }
    else {
        /*
        Some previous neighbor set has been encountered before, and is located at neighbor_set_traj[curr_deploying_agent_id - 1][neighs_encountered_before_idx].
        If curr_deploying_agent_curr_neighs has been encountered before, and was encountered just after the *previous* previously encountered
        neighbor set, we are going in loop.
        */
        int index_of_encounter = get_index_of_encounter(curr_deploying_agent_id, curr_deploying_agent_curr_neighs);
        if (index_of_encounter == neighs_encountered_before_idx + 1) {
            return true;
        }
        else {
            neighs_encountered_before_idx = index_of_encounter;
        }
    }
    return false;
}