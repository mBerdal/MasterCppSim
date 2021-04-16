#include "plotter.h"
#include "matplotlib-cpp/matplotlibcpp.h"
#include "helper.h"

namespace plt = matplotlibcpp;

void plot_line_segment(Vector2d start, Vector2d end, const map<string, string>& keywords = {{}}) {
  vector<double> x_range = {start(0), end(0)};
  vector<double> y_range = {start(1), end(1)};
  plt::plot(x_range, y_range, keywords);
}

void plot_config(Simulator simulator, string run_name) {
    plt::figure_size(500, 500);

    /*
    Plotting environment
    */
    for (Wall const &w : simulator.get_environment().get_walls()) {
        plot_line_segment(
          w.get_start(),
          w.get_end(),
          {{"color", "#8A8A8A"}});
    }

    vector<double> end_x_pos;
    vector<double> end_y_pos;
    for (int beacon_id = 0; beacon_id < simulator.get_num_deployed_beacons(); beacon_id++) {
        Vector2d beacon_final_pos = simulator.get_beacon_traj_data(beacon_id).topRightCorner(2, 1);
        end_x_pos.push_back(beacon_final_pos(0));
        end_y_pos.push_back(beacon_final_pos(1));

        plt::annotate(
          to_string(beacon_id),
          beacon_final_pos(0),
          beacon_final_pos(1)
        );

        /*
        Plotting beacon exploration direction
        */
        plot_line_segment(
          beacon_final_pos,
          beacon_final_pos + simulator.get_beacon_exploration_dir(beacon_id),
          {{"color", "green"}}
        );
        /*
        Plotting beacon trajectory
        */
        plt::plot(
            eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(beacon_id).row(Simulator::POSITION_X_IDX)),
            eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(beacon_id).row(Simulator::POSITION_Y_IDX)),
            {{"linestyle", "--"}, {"color", "#CDCDCD"}}
        );

    }

    /*
    Plotting points at final positions of the beacons
    */
    plt::scatter(
        end_x_pos,
        end_y_pos,
        40.0
    );

    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::axis("tight");

    if (run_name != "") {
      plt::save(FIGURES_DIR + run_name + "_config.eps");
    }
    plt::show(true);
}

void plot_agent_force_vs_time(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size(500, 500);

    vector<double> time = eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id).row(Simulator::TIMESTAMP_IDX));

    plt::plot(
      time,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_N_X_IDX, Simulator::F_N_Y_IDX), all).colwise().norm()),
      {{"label", "F_n"}}
    );

    plt::plot(
      time,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_E_X_IDX, Simulator::F_E_Y_IDX), all).colwise().norm()),
      {{"label", "F_e"}}
    );

    plt::plot(
      time,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_X_IDX, Simulator::F_Y_IDX), all).colwise().norm()),
      {{"label", "F"}}
    );

    plt::plot(
      time,
      vector<double>(time.size(), simulator.get_force_saturation_limit()),
      {{"label", "Saturation threshold"}, {"linestyle", "--"}, {"color", "black"}}
    );

    plt::xlabel("Time [s]");
    plt::ylabel("Force [N]");

    plt::legend();
    plt::axis("tight");

    if (run_name != "") {
      plt::save(FIGURES_DIR + run_name + "_force_v_time_agent_" + to_string(agent_id) + ".eps");
    }
    plt::show(true);
}

void plot_agent_force_vs_dist(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size(500, 500);

    vector<double> dist = eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::POSITION_X_IDX, Simulator::POSITION_Y_IDX), all).colwise().norm());

    plt::plot(
      dist,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_N_X_IDX, Simulator::F_N_Y_IDX), all).colwise().norm()),
      {{"label", "F_n"}}
    );

    plt::plot(
      dist,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_E_X_IDX, Simulator::F_E_Y_IDX), all).colwise().norm()),
      {{"label", "F_e"}}
    );

    plt::plot(
      dist,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_X_IDX, Simulator::F_Y_IDX), all).colwise().norm()),
      {{"label", "F"}}
    );

    plt::plot(
      dist,
      vector<double>(dist.size(), simulator.get_force_saturation_limit()),
      {{"label", "Saturation threshold"}, {"linestyle", "--"}, {"color", "black"}}
    );

    plt::xlabel("Distance [m]");
    plt::ylabel("Force [N]");

    plt::legend();
    plt::axis("tight");

    if (run_name != "") {
      plt::save(FIGURES_DIR + run_name + "_force_v_dist_agent_" + to_string(agent_id) + ".eps");
    }
    plt::show(true);
}

void plot_Xi_model() {
  double d_perf = 1;
  double d_none = 3;
  double xi_bar = 3;

  vector<double> dists;
  vector<double> xis;

  for (double d=0; d < d_none + 1; d += 0.1) {
    dists.push_back(d);
    xis.push_back(get_Xi_from_model(d, d_perf, d_none, xi_bar));
  }

  plt::figure_size(500, 500);
  plt::plot(dists, xis);
  plt::show(true);
}