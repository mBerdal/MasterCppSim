#include "env.h"
#include "sim.h"
#include "helper.h"
#include <iostream>
#include <list>
#include <sciplot/sciplot.hpp>

using namespace sciplot;
using namespace std;

Vector2d get_force_from_beacon(Vector2d agent_pos, int beacon_id, Vector2d beacon_pos, double beacon_xi) {
    double k_i = 2;
    double a_i = beacon_id + 1;
    return -k_i * (agent_pos - a_i * (beacon_pos + beacon_xi * Vector2d::UnitX()));
}

int main() {
    int n_agents = 1;
    int num_rays_per_range_sensor = 1;

    // Creating rectangular environment
    Matrix<double, 2, 4> m;
    m << -2, -2, 4, 4,
        -2, 2, 2, -2;
    list<Matrix<double, 2, Dynamic>> obstacles;
    obstacles.push_back(m);
    Env environment = Env();

    // Initializing simulator and running
    Simulator s(environment, n_agents, num_rays_per_range_sensor, *get_force_from_beacon);
    s.simulate(0.05);

    for (int i = 0; i < n_agents; i++) {
        cout << s.get_agent_traj_data()[i].topRightCorner(6, 1) << "\n-------\n";
    }

    Plot plt;
    plt.xlabel("x [m]");    
    plt.ylabel("y [m]");
    plt.palette("set2");


    plt.drawCurve(
        eig_mat2std_vec((MatrixXd) s.get_agent_traj_data()[0].row(0)), 
        eig_mat2std_vec((MatrixXd) s.get_agent_traj_data()[0].row(1))
    ).lineWidth(2).label("");
    plt.save("../file.svg");
}