#include "plotter.h"
#include "matplotlib-cpp/matplotlibcpp.h"
#include "helper.h"

#include  <iomanip>

namespace plt = matplotlibcpp;

string get_interpolated_color(double interpolator, interpolation_color_t gradient) {
  int r = floor(gradient.end_r*interpolator + gradient.base_r*(1-interpolator));
  int g = floor(gradient.end_g*interpolator + gradient.base_g*(1-interpolator));
  int b = floor(gradient.end_b*interpolator + gradient.base_b*(1-interpolator));

  stringstream ss;
  ss << "#"
  << setfill('0') << setw(2) << right << hex << r
  << setfill('0') << setw(2) << right << hex << g
  << setfill('0') << setw(2) << right << hex << b;
  return ss.str();
}

void plot_line_segment(Vector2d start, Vector2d end, const map<string, string>& keywords = {{}}) {
  vector<double> x_range = {start(0), end(0)};
  vector<double> y_range = {start(1), end(1)};
  plt::plot(x_range, y_range, keywords);
}

void plot_environment(Simulator simulator) {
    for (Wall const &w : simulator.get_environment().get_walls()) {
        plot_line_segment(
          w.get_start(),
          w.get_end(),
          {{"color", DARK_GRAY}}
        );
    }
}

void plot_single_beacon_traj(Simulator simulator, int beacon_id, bool show, bool add_legend, string run_name) {
    if (show) {
      plt::figure_size(500, 500);
      plot_environment(simulator);
    }
    
    MatrixXd beacon_traj_data = simulator.get_beacon_traj_data(beacon_id);
    Vector2d beacon_final_pos = beacon_traj_data.topRightCorner(2, 1);

    plt::scatter(
      vector<double>(1, beacon_final_pos(0)),
      vector<double>(1, beacon_final_pos(1)),
      SCATTER_DOT_SIZE,
      {{"color", BLUE}}
    );
    plt::annotate(
      to_string(beacon_id),
      beacon_final_pos(0),
      beacon_final_pos(1)
    );

    /*
    Plotting beacon exploration direction
    */
    /*
    plot_line_segment(
      beacon_final_pos,
      beacon_final_pos + simulator.get_beacon_exploration_dir(beacon_id, ExpVecType::OBS_AVOIDANCE),
      {{"color", RED}, {"label", add_legend ? R"($\mathbf{v}_{obs}$)" : ""}}
    );
    plot_line_segment(
      beacon_final_pos,
      beacon_final_pos + simulator.get_beacon_exploration_dir(beacon_id, ExpVecType::NEIGH_INDUCED),
      {{"color", GREEN}, {"label", add_legend ? R"($\mathbf{v}_{neighs}$)": ""}}
    );
    plot_line_segment(
      beacon_final_pos,
      beacon_final_pos + simulator.get_beacon_exploration_dir(beacon_id, ExpVecType::NEIGH_INDUCED_RANDOM),
      {{"color", PURPLE}, {"label",  add_legend ? R"($\mathbf{v}_{nom}$)" : ""}}
    );
    */

    plot_line_segment(
      beacon_final_pos,
      beacon_final_pos + simulator.get_applied_beacon_exploration_dir(beacon_id),
      {{"color", BLUE}, {"label", add_legend ? R"($\mathbf{v}$)" : ""}}
    );

    /*
    Plotting beacon obstacle avoidance vector
    plot_line_segment(
      beacon_final_pos,
      beacon_final_pos + beacon_traj_data.block(Simulator::O_HAT_X_IDX, beacon_traj_data.cols() - 1, 2, 1),
      {{"color", ORANGE}, {"label",  add_legend ? R"($\hat{\mathbf{o}}$)" : ""}}
    );
    */
    /*
    Plotting beacon trajectory
    */
    plt::plot(
        eig_vec2std_vec((VectorXd) beacon_traj_data.row(Simulator::POSITION_X_IDX)),
        eig_vec2std_vec((VectorXd) beacon_traj_data.row(Simulator::POSITION_Y_IDX)),
        {{"linestyle", "--"}, {"color", LITE_GRAY}}
    );

    if (run_name != "" || show) {
      plt::xlabel("x [m]");
      plt::ylabel("y [m]");
      plt::axis("tight");
      plt::legend({{"loc", "upper right"}});
    }

    if (run_name != "") {
      plt::axis("tight");
      plt::save(
        FIGURES_DIR + to_string(simulator.get_num_deployed_agents()) + "_agent_" + run_name + "_agent_" + to_string(beacon_id) + "_traj.eps"
      );
    }
    if (show) {
      plt::show(true);
    }
}

void plot_config(Simulator simulator, string run_name) {
    plt::figure_size(500, 500);

    plot_environment(simulator);
    

    for (int beacon_id = 0; beacon_id < simulator.get_num_deployed_beacons(); beacon_id++) {
      plot_single_beacon_traj(simulator, beacon_id, false, beacon_id == 0);
    }

    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::axis("tight");
    plt::legend({{"loc", "upper right"}});

    if (run_name != "") {
      plt::axis("tight");
      plt::save(
        FIGURES_DIR + to_string(simulator.get_num_deployed_agents()) + "_agent_" + run_name + "_config.eps"
      );
    }
    plt::show(true);
}


void plot_agent_force_vs_time(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size(500, 500);

    vector<double> time = eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id).row(Simulator::TIMESTAMP_IDX));

    plt::plot(
      time,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_N_X_IDX, Simulator::F_N_Y_IDX), all).colwise().norm()),
      {{"label", R"($||\mathbf{F}_{n}||$)"}}
    );

    plt::plot(
      time,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_E_X_IDX, Simulator::F_E_Y_IDX), all).colwise().norm()),
      {{"label", R"($||\mathbf{F}_{e}||$)"}}
    );

    plt::plot(
      time,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_X_IDX, Simulator::F_Y_IDX), all).colwise().norm()),
      {{"label", R"($||\mathbf{F}||$)"}}
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
      plt::axis("tight");
        plt::save(
          FIGURES_DIR + run_name + "_force_v_time_agent_" + to_string(agent_id) + ".eps"
        );
    }
    plt::show(true);
}

void plot_agent_force_vs_dist(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size(500, 500);

    vector<double> dist = eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::POSITION_X_IDX, Simulator::POSITION_Y_IDX), all).colwise().norm());

    plt::plot(
      dist,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_N_X_IDX, Simulator::F_N_Y_IDX), all).colwise().norm()),
      {{"label", R"(||\mathbf{F}_{n}||)"}}
    );

    plt::plot(
      dist,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_E_X_IDX, Simulator::F_E_Y_IDX), all).colwise().norm()),
      {{"label", R"(||\mathbf{F}_{e}||)"}}
    );

    plt::plot(
      dist,
      eig_vec2std_vec((VectorXd) simulator.get_beacon_traj_data(agent_id)(seq(Simulator::F_X_IDX, Simulator::F_Y_IDX), all).colwise().norm()),
      {{"label", R"(||\mathbf{F}||)"}}
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
      plt::axis("tight");
      plt::save(
        FIGURES_DIR + run_name + "_force_v_dist_agent_" + to_string(agent_id) + ".eps"
      );
    }
    plt::show(true);
}

void plot_agent_neigh_traj(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size((16 / 9.0) * 500, 500);
    vector<tuple<double, vector<int>>> agent_neigh_traj = simulator.get_agent_neigh_traj(agent_id);
    stringstream stream;
    bool agent_caught_looping = simulator.agent_id_neigh_traj_idx_of_loop_start_end_map.find(agent_id) != simulator.agent_id_neigh_traj_idx_of_loop_start_end_map.end();
    string clr;
    for (int i = 0; i < agent_neigh_traj.size(); i++) {
      clr = "";
      if (agent_caught_looping){
        if (i == simulator.agent_id_neigh_traj_idx_of_loop_start_end_map[agent_id].first || i == simulator.agent_id_neigh_traj_idx_of_loop_start_end_map[agent_id].second) {
          clr = "red";
        }
        else if (i == simulator.agent_id_neigh_traj_idx_of_loop_start_end_map[agent_id].first + 1 || i == simulator.agent_id_neigh_traj_idx_of_loop_start_end_map[agent_id].second + 1) {
          clr = "green";
        }
      } 
      for (const int & neigh_id : get<1>(agent_neigh_traj[i])) {
        if (clr != "red" && clr != "green") {
          clr = get_interpolated_color(neigh_id / (double) (agent_id - 1), DAT_GRADIENT);
        }
        plt::scatter(
          vector<double>(1, get<0>(agent_neigh_traj[i])),
          vector<int>(1, neigh_id),
          SCATTER_DOT_SIZE,
          {{"color", clr}}
        );
      }
    }
    plt::ylim(-1, agent_id);
    plt::xlabel(R"(Elapsed time (since $\nu_{1}$ deployed) [s])");
    plt::ylabel("ID of neighboring beacon");
    plt::title(R"(Neighbor set evolution for $\nu_{)" + to_string(agent_id) + R"(}$)");
    if (run_name != "") {
      plt::axis("tight");
      plt::save(
        FIGURES_DIR + run_name + "_neigh_traj_agent_" + to_string(agent_id) + ".eps"
      );
    }
    plt::show();
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