#include "include/sim.h"
#include "include/range_ray.h"

#include <iostream> /* cout */
#include <math.h>   /* pow */

namespace eig = Eigen;
using namespace std;

Simulator::Simulator(double base_dt, double gain_factor, double k_obs, Env environment, int num_agents_to_deploy, int num_rays_per_range_sensor,
                    XiParams xi_params, eig::Vector2d (*get_force_func)(double, eig::Vector2d, eig::Vector2d, double, double),
                    double force_saturation_limit, double minimum_force_threshold, int agent_max_steps,
                    ExpVecType use_exp_vec_type) {

    this->base_dt = base_dt;
    this->gain_factor = gain_factor;
    this->k_obs = k_obs;
    RangeRay::env = environment;
    this->environment = environment;
    this->num_agents_to_deploy = num_agents_to_deploy;
    this->get_force_func = get_force_func;
    this->force_saturation_limit = force_saturation_limit;
    this->minimum_force_threshold = minimum_force_threshold;
    this->agent_max_steps = agent_max_steps;
    this->use_exp_vec_type = use_exp_vec_type;

    beacon_traj_data = new eig::MatrixXd[1 + num_agents_to_deploy];
    beacon_traj_data[0] = eig::Matrix<double, NUM_TRAJ_DATA_POINTS, 1>::Zero();

    uniformity_traj = new double[1 + num_agents_to_deploy];
    uniformity_traj[0] = 0;

    neighbor_set_traj = new vector<pair<double, vector<int>>>[num_agents_to_deploy];

    exploration_angles = new map<ExpVecType, vector<double>>[1 + num_agents_to_deploy];
    set_all_exp_vec_types_for_beacon(0, {}, eig::Vector2d::Zero());


    // Initialize range sensors
    this->num_rays_per_range_sensor = num_rays_per_range_sensor;
    if (num_rays_per_range_sensor == 1) {
        ray_angles_rel_SENSOR = eig::ArrayXd::Zero(1);
    } else {
        ray_angles_rel_SENSOR = eig::ArrayXd::LinSpaced(num_rays_per_range_sensor, -RANGE_SENSOR_FOV_RAD / 2.0, RANGE_SENSOR_FOV_RAD / 2.0);
    }

    this->xi_params = xi_params;
    RandomNumberGenerator::seed();
    agent_id_to_loop_initiators_map = map<int, vector<eig::Vector2i>>();
}

void Simulator::simulate() {
    time = 0;
    double dt;
    
    for (int curr_deploying_agent_id = 1; curr_deploying_agent_id <= num_agents_to_deploy; curr_deploying_agent_id++) {
        
        beacon_traj_data[curr_deploying_agent_id] = eig::MatrixXd();
        beacon_traj_data[curr_deploying_agent_id].conservativeResize(NUM_TRAJ_DATA_POINTS, TRAJ_DATA_SECTOR_SIZE);

        beacon_traj_data[curr_deploying_agent_id].topLeftCorner(NUM_STATE_VARIABLES, 1) = eig::Vector<double, NUM_STATE_VARIABLES>::Zero();
        beacon_traj_data[curr_deploying_agent_id].block(F_N_X_IDX, 0, 6, 1) = eig::Vector<double, 6>::Zero();
        beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, 0) = time;

        int step_count = 0;
        neigh_set_repeat_look_back_horizon = 0;
        most_recent_neigh_set_repeat_indices = eig::Vector2i::Zero();
        
        StepResult step_result = NO_PROBLEM;
        while (step_result == NO_PROBLEM && step_count < agent_max_steps) {
            dt = base_dt;
            step_result = do_step(curr_deploying_agent_id, &dt, step_count);
            time += dt;
            step_count++;
        }

        cout << "Agent " << curr_deploying_agent_id << " landed due to ";

        /* Removing unused data columns
        
        If deployment was haulted due to lack of neighbors before performing step,
        we pretend that the step causing the drone to loose contact never happened.
        If deployment was haulted due to a lack of force, we are happy and include
        the step taken before haulting due to a lack of force.
        */
        if (step_result == NO_NEIGHBORS) {
            step_count--;
            time -= dt;
            cout << "a lack of neighbors, ";
        } else {
            cout << "a lack of force, ";
        }

        eig::MatrixXd valid_traj_data = beacon_traj_data[curr_deploying_agent_id].leftCols(step_count);
        beacon_traj_data[curr_deploying_agent_id] = valid_traj_data;

        compute_beacon_exploration_dir(curr_deploying_agent_id, curr_deploying_agent_id - 1);

        cout << "in " << step_count << " steps, with " << neighbor_set_traj[curr_deploying_agent_id - 1].back().second.size() << " neighbors.\n";

        compute_uniformity(curr_deploying_agent_id);
    }
}

void Simulator::compute_uniformity(int max_agent_id) {
    double tmp = 0;
    for (int beacon_id = 0; beacon_id <= max_agent_id; beacon_id++) {
        eig::Vector2d beacon_pos = beacon_traj_data[beacon_id].topRightCorner(2, 1);
        vector<int> beacon_neighs = get_beacon_neighbors(beacon_id, beacon_pos , max_agent_id);
        int num_beacon_neighs = beacon_neighs.size();

        double beacon_neigh_avg_dist = 0;
        for (const int & neigh_id : beacon_neighs) {
            eig::Vector2d neigh_pos = beacon_traj_data[neigh_id].topRightCorner(2, 1);
            beacon_neigh_avg_dist += (beacon_pos - neigh_pos).norm();
        }
        beacon_neigh_avg_dist /= (double) num_beacon_neighs;

        double sum_of_squared_deviance = 0;
        for (const int & neigh_id : beacon_neighs) {
            eig::Vector2d neigh_pos = beacon_traj_data[neigh_id].topRightCorner(2, 1);
            sum_of_squared_deviance += pow((beacon_pos - neigh_pos).norm() - beacon_neigh_avg_dist, 2);
        }

        double beacon_uniformity = sqrt((1 / (double) num_beacon_neighs) * sum_of_squared_deviance);
        tmp += beacon_uniformity;
    }
    uniformity_traj[max_agent_id] = tmp / (double) (max_agent_id + 1);
}



Simulator::StepResult Simulator::do_step(int curr_deploying_agent_id, double* dt_ptr, int step_count) {
    if (step_count + 1 >= beacon_traj_data[curr_deploying_agent_id].cols()) {
        beacon_traj_data[curr_deploying_agent_id].conservativeResize(
            NUM_TRAJ_DATA_POINTS,
            beacon_traj_data[curr_deploying_agent_id].cols() + TRAJ_DATA_SECTOR_SIZE
        );
    }

    eig::Vector<double, NUM_STATE_VARIABLES> curr_deploying_agent_state = beacon_traj_data[curr_deploying_agent_id].block(0, step_count, NUM_STATE_VARIABLES, 1);
    eig::Vector2d curr_deploying_agent_pos = curr_deploying_agent_state.head(2);
    double curr_deploying_agent_yaw = curr_deploying_agent_state(YAW_IDX);

    vector<int> curr_deploying_agent_neighs = get_beacon_neighbors(
        curr_deploying_agent_id,
        curr_deploying_agent_pos,
        curr_deploying_agent_id - 1
    );

    if (curr_deploying_agent_neighs.size() == 0) {
        return NO_NEIGHBORS;
    }


    eig::Vector2d F_n = get_neigh_force_on_agent(
        curr_deploying_agent_pos,
        curr_deploying_agent_neighs
    );

    eig::Vector2d o_hat = get_obstacle_avoidance_vector(curr_deploying_agent_pos, curr_deploying_agent_yaw);
    eig::Vector2d F_e = get_env_force_agent(o_hat);

    eig::Vector2d F_nominal = F_n + F_e;
    eig::Vector2d F = clamp_vec(F_nominal, force_saturation_limit);
    if (F.norm() < minimum_force_threshold) {
        return ZERO_FORCE;
    }
    
    eig::Vector<double, NUM_STATE_VARIABLES> state_der;
    state_der << F(0), F(1), 0, 0, 0;

    LoopCheckResult loop_check_result = LoopCheckResult::NO_LOOP;
    if (step_count == 0 || did_neigh_set_change(curr_deploying_agent_id, curr_deploying_agent_neighs)) {
        neighbor_set_traj[curr_deploying_agent_id - 1].push_back(
            pair<double, vector<int>>(
                beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, step_count),
                curr_deploying_agent_neighs
            )
        );

        loop_check_result = get_loop_check_result(curr_deploying_agent_id);
        if (loop_check_result == LoopCheckResult::LOOP) {
            for (const int & neighbor_id : curr_deploying_agent_neighs) {
                compute_beacon_exploration_dir(neighbor_id, curr_deploying_agent_id - 1);
            }

            agent_id_to_loop_initiators_map[curr_deploying_agent_id].push_back(
                most_recent_neigh_set_repeat_indices
            );

            neigh_set_repeat_look_back_horizon = most_recent_neigh_set_repeat_indices(1);
        }
    }


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
    if (step_count > 2000) {
        *dt_ptr = base_dt / 10.0;
    }
    else if (step_count > 10000 || loop_check_result == LoopCheckResult::JITTER) {
        *dt_ptr = base_dt / 10000.0;
    }

    beacon_traj_data[curr_deploying_agent_id].block(0, step_count + 1, NUM_STATE_VARIABLES, 1) = \
    curr_deploying_agent_state + (*dt_ptr) * state_der;

    beacon_traj_data[curr_deploying_agent_id].block(F_N_X_IDX, step_count + 1, 2, 1) = F_n;
    beacon_traj_data[curr_deploying_agent_id].block(F_E_X_IDX, step_count + 1, 2, 1) = F_e;
    beacon_traj_data[curr_deploying_agent_id].block(F_X_IDX, step_count + 1, 2, 1) = F_nominal;
    beacon_traj_data[curr_deploying_agent_id].block(O_HAT_X_IDX, step_count + 1, 2, 1) = o_hat;
    beacon_traj_data[curr_deploying_agent_id](TIMESTAMP_IDX, step_count + 1) = time;
    
    return NO_PROBLEM;
}

void Simulator::compute_beacon_exploration_dir(int beacon_id, int max_neigh_id){
    vector<int> beacon_neighs = get_beacon_neighbors(
        beacon_id,
        beacon_traj_data[beacon_id].topRightCorner(2, 1),
        max_neigh_id
    );
    eig::Vector2d neigh_o_hat = beacon_traj_data[beacon_id].block(
        O_HAT_X_IDX,
        beacon_traj_data[beacon_id].cols()-1,
        2,
        1
    );
    set_all_exp_vec_types_for_beacon(beacon_id, beacon_neighs, neigh_o_hat);
}

double Simulator::get_beacon_nominal_weight(int beacon_id) const {
    return pow(gain_factor, beacon_id);
}

eig::Vector2d Simulator::get_neigh_force_on_agent(eig::Vector2d agent_pos, vector<int> agent_curr_neighs) const {
    eig::Vector2d F = eig::Vector2d::Zero();

    double sum_of_weights = 0;
    for (int beacon_id : agent_curr_neighs) {
        sum_of_weights += get_beacon_nominal_weight(beacon_id);
    }

    for (int beacon_id : agent_curr_neighs) {
        eig::Vector2d other_beacon_pos = beacon_traj_data[beacon_id].topRightCorner(2, 1);

        double dist = (agent_pos - other_beacon_pos).norm();
        double xi = get_Xi_from_model(dist, xi_params.d_perf, xi_params.d_none, xi_params.xi_bar);

        double k_i = (get_beacon_nominal_weight(beacon_id)) / sum_of_weights;

        F += (*get_force_func)(k_i, agent_pos, other_beacon_pos, exploration_angles[beacon_id][use_exp_vec_type].back(), xi);
    }
    return F;
}

eig::Vector2d Simulator::get_env_force_agent(eig::Vector2d obstacle_avoidance_vec) const {
    return k_obs * (1 / (RANGE_SENSOR_MAX_RANGE_METERS - obstacle_avoidance_vec.norm())) * obstacle_avoidance_vec;
}

eig::Vector2d Simulator::get_obstacle_avoidance_vector(eig::Vector2d agent_pos, double agent_yaw) const {
    eig::Matrix<double, 4, 2> sensed_ranges_and_angles = get_sensed_ranges_and_angles(
        agent_pos,
        agent_yaw
    );
    eig::Vector2d o = eig::Vector2d::Zero();
    for (int i = 0; i < 4; i++) {
        o += sensed_ranges_and_angles(i, 0) * eig::Rotation2D<double>(sensed_ranges_and_angles(i, 1)).toRotationMatrix() * eig::Vector2d::UnitX();
    }

    double o_norm = o.norm();
    return o_norm <= RANGE_SENSOR_MAX_RANGE_METERS ? o : (RANGE_SENSOR_MAX_RANGE_METERS / o_norm)*o;
}

eig::Matrix<double, 4, 2> Simulator::get_sensed_ranges_and_angles(eig::Vector2d agent_pos, double agent_yaw) const {

    eig::Matrix<double, 4, 2> sensed_ranges_and_angles;
    sensed_ranges_and_angles.col(0) = ((double)RANGE_SENSOR_MAX_RANGE_METERS) * eig::Vector4d::Ones();
    sensed_ranges_and_angles.col(1) = agent_yaw * eig::Vector4d::Ones() + eig::Vector4d::LinSpaced(0, (3 / 2.0) * M_PI);

    for (int i = 0; i < 4; i++) {
        double sensor_angle_rel_NED = agent_yaw + (double)i * M_PI / 2.0;
        eig::ArrayXd ray_angles_rel_NED = ray_angles_rel_SENSOR + sensor_angle_rel_NED;

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

vector<int> Simulator::get_beacon_neighbors(int beacon_id, eig::Vector2d beacon_pos, int max_neigh_id) const {
    vector<int> neighs;
    for (int other_beacon_id=0; other_beacon_id <= max_neigh_id; other_beacon_id++) {
        if (other_beacon_id != beacon_id) {
            eig::Vector2d other_beacon_pos = beacon_traj_data[other_beacon_id].topRightCorner(2, 1);
            double dist = (beacon_pos - other_beacon_pos).norm();
            if (get_Xi_from_model(dist, xi_params.d_perf, xi_params.d_none, xi_params.xi_bar) > xi_params.neigh_treshold) {
                neighs.push_back(other_beacon_id);
            }
        }
    }
    return neighs;
}

CircleSector Simulator::get_exploration_sector(int curr_deploying_agent_id, vector<int> agent_neighbors, eig::Vector2d obstacle_avoidance_vec) const {
    if (agent_neighbors.size() == 0) {
        cout << "No neighbors for agent " << curr_deploying_agent_id << ". Using 0 as bisection based angle\n";
        return CircleSector(0, 0);
    }
    int num_neighs = agent_neighbors.size();
    vector<double> angles;
    double sector_min_bound, sector_max_bound;
    bool avoid_obstacle = obstacle_avoidance_vec.norm() >= RANGE_SENSOR_MAX_RANGE_METERS / 2.0;


    if (avoid_obstacle) {
        double obstacle_avoidance_angle = atan2(obstacle_avoidance_vec(1), obstacle_avoidance_vec(0));
        sector_max_bound = clamp_zero_two_pi(obstacle_avoidance_angle + M_PI_2);
        sector_min_bound = clamp_zero_two_pi(obstacle_avoidance_angle - M_PI_2);
        angles.push_back(sector_max_bound);
        angles.push_back(sector_min_bound);
    }

    for (int i = 0; i < num_neighs; i++) {
        eig::Vector2d vec_to_neigh = (
            beacon_traj_data[agent_neighbors[i]].topRightCorner(2, 1) - 
            beacon_traj_data[curr_deploying_agent_id].topRightCorner(2, 1)
        );
        angles.push_back(
            clamp_zero_two_pi(atan2(vec_to_neigh(1), vec_to_neigh(0)))
        );
    }
    int num_sectors = num_neighs + (avoid_obstacle ? 2 : 0);
    if(num_sectors == 1) {
        return CircleSector(angles[0], angles[0] + 2 * M_PI);
    }

    sort(angles.begin(), angles.end());
    vector<CircleSector> invalid_sectors;
    vector<CircleSector> valid_sectors;
    for (int i = 0; i < num_sectors; i++) {
        bool is_valid_sector = true;
        double sector_start = (i < num_sectors - 1) ? angles[i] : angles[num_sectors - 1];
        double sector_end = (i < num_sectors - 1) ? angles[i + 1] : angles[0];

        if (avoid_obstacle) {
            is_valid_sector = (sector_max_bound > sector_min_bound) ? sector_start >= sector_min_bound && sector_start < sector_max_bound : sector_start >= sector_min_bound || sector_start < sector_max_bound;
        }
        if (is_valid_sector) {
            valid_sectors.push_back(CircleSector(sector_start, sector_end));
        }
        else {
            invalid_sectors.push_back(CircleSector(sector_start, sector_end));
        }
    }

    CircleSector valid_sector_with_max_central_angle = *max_element(
        valid_sectors.begin(),
        valid_sectors.end(),
        CircleSector::cmp
    );
    return valid_sector_with_max_central_angle;
}

void Simulator::set_all_exp_vec_types_for_beacon(int beacon_id, vector<int> neighbor_ids, eig::Vector2d obstacle_avoidance_vec){
    CircleSector exploration_sector = get_exploration_sector(beacon_id, neighbor_ids, obstacle_avoidance_vec);

    exploration_angles[beacon_id][ExpVecType::NEIGH_INDUCED].push_back(
        exploration_sector.get_angle_bisector()
    );
    exploration_angles[beacon_id][ExpVecType::NEIGH_INDUCED_RANDOM].push_back(
        exploration_sector.get_angle_bisector() + RandomNumberGenerator::get_between(-1, 1) * (exploration_sector.get_central_angle() / 4.0)
    );
    exploration_angles[beacon_id][ExpVecType::TOTAL].push_back(get_beacon_exploration_angles(beacon_id, NEIGH_INDUCED).back());
}

double Simulator::get_avg_angle_away_from_neighs(int beacon_id, vector<int> neighbor_ids) const {
    if (neighbor_ids.size() == 0) {
        cout << "Beacon " << beacon_id << " has no neighbors. Using zero as exploration angle.\n";
        return 0;
    }
    eig::Vector2d avg_vec_to_neighs = eig::Vector2d::Zero();
    for (int neigh_id : neighbor_ids) {
        avg_vec_to_neighs += (
            beacon_traj_data[beacon_id].topRightCorner(2, 1) - beacon_traj_data[neigh_id].topRightCorner(2, 1)
        ).normalized();
    }
    return atan2(avg_vec_to_neighs(1), avg_vec_to_neighs(0));
}

int Simulator::get_curr_neigh_set_index_of_previous_encounter(int curr_deploying_agent_id) const {
    for (int i = neighbor_set_traj[curr_deploying_agent_id - 1].size() - 2; i >= neigh_set_repeat_look_back_horizon; i--) {
        vector<int> recorded_neigh_set = neighbor_set_traj[curr_deploying_agent_id - 1][i].second;
        if (vectors_equal(recorded_neigh_set, neighbor_set_traj[curr_deploying_agent_id - 1].back().second)) {
            return i;
        }
    }
    // Current neighbor set never encountered between the current one and the horizon
    return -1;
}


Simulator::LoopCheckResult Simulator::get_loop_check_result(int curr_deploying_agent_id) {
    /*if (neighs_encountered_before_idx == -1) {
        neighs_encountered_before_idx = get_curr_neigh_set_index_of_previous_encounter(curr_deploying_agent_id);
    }
    else {
        int index_of_encounter = get_curr_neigh_set_index_of_previous_encounter(curr_deploying_agent_id);
        if (index_of_encounter == neighs_encountered_before_idx + 1) {
            return true;
        }
        else {
            neighs_encountered_before_idx = index_of_encounter;
        }
    }
    return false;
    */
    LoopCheckResult result = LoopCheckResult::NO_LOOP;
    if (most_recent_neigh_set_repeat_indices.sum() == 0) {
        int index_of_previous_encounter = get_curr_neigh_set_index_of_previous_encounter(curr_deploying_agent_id);
        if (index_of_previous_encounter != -1) {
            most_recent_neigh_set_repeat_indices << index_of_previous_encounter, neighbor_set_traj[curr_deploying_agent_id - 1].size() - 1;
        }
    }
    else {
        int index_of_previous_encounter = get_curr_neigh_set_index_of_previous_encounter(curr_deploying_agent_id);
        if (index_of_previous_encounter != -1) {
            eig::Vector2i tmp;
            tmp << index_of_previous_encounter, neighbor_set_traj[curr_deploying_agent_id - 1].size() - 1;

            if (most_recent_neigh_set_repeat_indices + eig::Vector2i::Ones() == tmp) {
                if (tmp(1) - tmp(0) > 2) {
                    result = LoopCheckResult::LOOP;
                } else {
                    result = LoopCheckResult::JITTER;
                }
            }
            most_recent_neigh_set_repeat_indices = tmp;
        }
    }
    return result;
}

double Simulator::get_wall_adjusted_angle(double nominal_angle, eig::Vector2d obstacle_avoidance_vec) const {
    double o_hat_angle = atan2(obstacle_avoidance_vec(1), obstacle_avoidance_vec(0));
    double o_hat_norm = obstacle_avoidance_vec.norm();

    double w_ang = o_hat_norm / RANGE_SENSOR_MAX_RANGE_METERS;
    double c = (1 - w_ang)*cos(nominal_angle) + w_ang*cos(o_hat_angle);
    double s = (1 - w_ang)*sin(nominal_angle) + w_ang*sin(o_hat_angle);
    return atan2(s, c);
}