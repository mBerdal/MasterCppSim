#include "plotter.h"
#include "helper.h"
#include "env.h"

#include "matplotlib-cpp/matplotlibcpp.h"

#include <iomanip>
#include <string>
#include <iostream>

#define FIGURES_DIR "../../figures/"

struct interpolation_color_t {
  int base_r, base_g, base_b, end_r, end_g, end_b;
};

#define DAT_GRADIENT interpolation_color_t{0, 199, 199, 230, 0, 145}
#define DAT_OTHER_GRADIENT interpolation_color_t{58, 168, 50, 181, 22, 22}

#define BLUE "#5865b8"
#define GREEN "#3d8045"
#define DARK_GRAY "#8A8A8A"
#define LITE_GRAY "#CDCDCD" 
#define RED "#d12a2a"
#define ORANGE "#ff9d00"
#define PURPLE "#c4169c"

#define SCATTER_DOT_SIZE 40.0

namespace eig = Eigen;
namespace plt = matplotlibcpp;
using namespace std;

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

void plot_line_segment(eig::Vector2d start, eig::Vector2d end, const map<string, string>& keywords = {{}}) {
  vector<double> x_range = {start(0), end(0)};
  vector<double> y_range = {start(1), end(1)};
  plt::plot(x_range, y_range, keywords);
}

void plot_environment(Simulator simulator, bool show) {
  if (show) {
    plt::figure_size(500, 500);
  }
  for (Wall const &w : simulator.get_environment().get_walls()) {
      plot_line_segment(
        w.get_start(),
        w.get_end(),
        {{"color", DARK_GRAY}}
      );
  }
  if (show) {
    plt::show();
  }
}

void plot_single_beacon_traj(Simulator simulator, int beacon_id, bool show, bool add_legend, string run_name) {
    if (show) {
      plt::figure_size(500, 500);
      plot_environment(simulator, false);
    }
    
    eig::MatrixXd beacon_traj_data = simulator.get_beacon_traj_data(beacon_id);
    eig::Vector2d beacon_final_pos = beacon_traj_data.topRightCorner(2, 1);

    /*
    Plotting beacon trajectory
    */
    plt::plot(
        eig_vec2std_vec((eig::VectorXd) beacon_traj_data.row(Simulator::POSITION_X_IDX)),
        eig_vec2std_vec((eig::VectorXd) beacon_traj_data.row(Simulator::POSITION_Y_IDX)),
        {{"linestyle", "--"}, {"color", LITE_GRAY}, {"alpha", "0,7"}}
    );

    /*
    Plotting beacon final position
    */

    plt::scatter(
      vector<double>(1, beacon_final_pos(0)),
      vector<double>(1, beacon_final_pos(1)),
      SCATTER_DOT_SIZE,
      {{"color", BLUE}, {"zorder", "100"}}
    );
    plt::annotate(
      to_string(beacon_id),
      beacon_final_pos(0),
      beacon_final_pos(1)
    );

    /*
    Plotting beacon exploration direction
    */
    int num_exp_vecs = simulator.get_applied_beacon_exploration_angles(beacon_id).size();
    for (int i = 0; i < num_exp_vecs; i++) {
      plot_line_segment(
        beacon_final_pos,
        beacon_final_pos + eig::Rotation2Dd(simulator.get_beacon_exploration_angles(beacon_id, ExpVecType::NEIGH_INDUCED)[i]).toRotationMatrix() * eig::Vector2d::UnitX(),
        {
          {"color", get_interpolated_color(i / (double) num_exp_vecs, DAT_OTHER_GRADIENT)},
          {"label", add_legend ? R"($\mathbf{v}_{n}$)" : ""}
        }
      );
      plot_line_segment(
        beacon_final_pos,
        beacon_final_pos + eig::Rotation2Dd(simulator.get_applied_beacon_exploration_angles(beacon_id)[i]).toRotationMatrix() * eig::Vector2d::UnitX(),
        {
          {"color", get_interpolated_color(i / (double) num_exp_vecs, DAT_GRADIENT)},
          {"label", add_legend ? R"($\mathbf{v}$)" : ""}
        }
      );
    }

    if (run_name != "" || show) {
      plt::xlabel("x [m]");
      plt::ylabel("y [m]");
      plt::axis("tight");
      plt::legend({{"loc", "upper right"}});
    }

    if (run_name != "") {
      plt::axis("tight");
      plt::save(
        FIGURES_DIR + to_string(simulator.get_num_deployed_agents()) + "_agent_" + run_name + "_agent_" + to_string(beacon_id) + "_traj.pdf"
      );
    }
    if (show) {
      plt::show(true);
    }
}

void plot_config(Simulator simulator, string run_name) {
    plt::figure_size(750, 750);

    plot_environment(simulator, false);
    

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
        FIGURES_DIR + to_string(simulator.get_num_deployed_agents()) + "_agent_" + run_name + "_config.pdf"
      );
    }
    plt::show(true);
}


void plot_agent_force_vs_time(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size(500, 500);

    vector<double> time = eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).row(Simulator::TIMESTAMP_IDX));

    plt::plot(
      time,
      eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).middleRows(Simulator::F_N_X_IDX, 2).colwise().norm()),
      {{"label", R"($||\mathbf{F}_{n}||$)"}}
    );

    plt::plot(
      time,
      eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).middleRows(Simulator::F_E_X_IDX, 2).colwise().norm()),
      {{"label", R"($||\mathbf{F}_{e}||$)"}}
    );

    plt::plot(
      time,
      eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).middleRows(Simulator::F_X_IDX, 2).colwise().norm()),
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
          FIGURES_DIR + run_name + "_force_v_time_agent_" + to_string(agent_id) + ".pdf"
        );
    }
    plt::show(true);
}

void plot_agent_force_vs_dist(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size(500, 500);

    vector<double> dist = eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).middleRows(Simulator::POSITION_X_IDX, 2).colwise().norm());

    plt::plot(
      dist,
      eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).middleRows(Simulator::F_N_X_IDX, 2).colwise().norm()),
      {{"label", R"(||\mathbf{F}_{n}||)"}}
    );

    plt::plot(
      dist,
      eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).middleRows(Simulator::F_E_X_IDX, 2).colwise().norm()),
      {{"label", R"(||\mathbf{F}_{e}||)"}}
    );

    plt::plot(
      dist,
      eig_vec2std_vec((eig::VectorXd) simulator.get_beacon_traj_data(agent_id).middleRows(Simulator::F_X_IDX, 2).colwise().norm()),
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
        FIGURES_DIR + run_name + "_force_v_dist_agent_" + to_string(agent_id) + ".pdf"
      );
    }
    plt::show(true);
}

void plot_agent_neigh_traj(Simulator simulator, int agent_id, string run_name) {
    plt::figure_size((16 / 9.0) * 500, 500);
    vector<pair<double, vector<int>>> agent_neigh_traj = simulator.get_agent_neigh_traj(agent_id);
    for (int i = 0; i < agent_neigh_traj.size(); i++) {
      for (const int & neigh_id : agent_neigh_traj[i].second) {
        plt::scatter(
          vector<double>(1, agent_neigh_traj[i].first),
          vector<int>(1, neigh_id),
          SCATTER_DOT_SIZE,
          {{"color", get_interpolated_color(neigh_id / (double) (agent_id - 1), DAT_GRADIENT)}}
        );
      }
    }
    if (simulator.agent_id_neigh_traj_idx_of_loop_start_end_map.count(agent_id) != 0) {
      /*
      Agent detected loop(s)
      */
      vector<pair<int, int>> loop_start_end = simulator.agent_id_neigh_traj_idx_of_loop_start_end_map[agent_id];
      for (const pair<int, int> & start_end : loop_start_end) {
        plt::scatter(
          vector<double>(agent_neigh_traj[start_end.first].second.size(), agent_neigh_traj[start_end.first].first),
          agent_neigh_traj[start_end.first].second,
          SCATTER_DOT_SIZE,
          {{"color", "red"}}
        );
        plt::scatter(
          vector<double>(agent_neigh_traj[start_end.first + 1].second.size(), agent_neigh_traj[start_end.first + 1].first),
          agent_neigh_traj[start_end.first + 1].second,
          SCATTER_DOT_SIZE,
          {{"color", "green"}}
        );
        plt::scatter(
          vector<double>(agent_neigh_traj[start_end.second].second.size(), agent_neigh_traj[start_end.second].first),
          agent_neigh_traj[start_end.second].second,
          SCATTER_DOT_SIZE,
          {{"color", "red"}}
        );
        plt::scatter(
          vector<double>(agent_neigh_traj[start_end.second + 1].second.size(), agent_neigh_traj[start_end.second + 1].first),
          agent_neigh_traj[start_end.second + 1].second,
          SCATTER_DOT_SIZE,
          {{"color", "green"}}
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
        FIGURES_DIR + run_name + "_neigh_traj_agent_" + to_string(agent_id) + ".pdf"
      );
    }
    plt::show();
}

void plot_uniformity_traj(Simulator simulator, string run_name) {
  plt::figure_size((16 / 9.0) * 400, 400);
  vector<double> num_agents;
  vector<double> uniformity;
  for (int num_deployed_agents= 0; num_deployed_agents < simulator.get_num_deployed_beacons(); num_deployed_agents++) {
    num_agents.push_back(num_deployed_agents);
    uniformity.push_back(simulator.get_uniformity_after_deploying_num_agents(num_deployed_agents));
  }

  plt::plot(
    num_agents,
    uniformity,
    {{"color", BLUE}, {"marker", "o"}}
  );
  plt::xlabel("Number of deployed agents");
  plt::ylabel("Uniformity");
  plt::title("Uniformity evolution");
  if (run_name != "") {
      plt::axis("tight");
      plt::save(
        FIGURES_DIR + run_name + "_uniformity_traj_" + to_string(simulator.get_num_deployed_agents()) + "_agent.pdf"
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

void plot_sector(CircleSector sector, string clr) {
  vector<double> perimeter_x;
  vector<double> perimeter_y;

  double start_ang = sector.start();
  double end_ang = sector.end() + (start_ang > end_ang ? 2 * M_PI : 0);

  for (double ang = start_ang; ang < end_ang; ang += 0.01) {
    perimeter_x.push_back(cos(ang));
    perimeter_y.push_back(sin(ang));
  }
  perimeter_x.push_back(cos(end_ang));
  perimeter_y.push_back(sin(end_ang));
  plt::plot(
    perimeter_x,
    perimeter_y,
    {{"color", clr}, {"linewidth", "2"}}
  );

  vector<double> sector_x;
  vector<double> sector_y;
  for (double r = 0; r < 1; r += 0.01) {
    sector_x.push_back(r*cos(start_ang));
    sector_y.push_back(r*sin(start_ang));
  }
  plt::plot(
    sector_x,
    sector_y,
    {{"color", "black"}, {"linestyle", "--"}}
  );
}

void plot_sectors(int beacon_id, vector<CircleSector> valid_sectors, vector<CircleSector> invalid_sectors, eig::Vector2d o_hat) {
  vector<double> circle_points_x;
  vector<double> circle_points_y;
  for (const CircleSector sector : valid_sectors) {
    plot_sector(sector, GREEN);
    circle_points_x.push_back(cos(sector.start()));
    circle_points_x.push_back(cos(sector.end()));
    circle_points_y.push_back(sin(sector.start()));
    circle_points_y.push_back(sin(sector.end()));
  }
  for (const CircleSector sector : invalid_sectors) {
    plot_sector(sector, RED);
    circle_points_x.push_back(cos(sector.start()));
    circle_points_x.push_back(cos(sector.end()));
    circle_points_y.push_back(sin(sector.start()));
    circle_points_y.push_back(sin(sector.end()));
  }
  plt::scatter(
    circle_points_x,
    circle_points_y,
    SCATTER_DOT_SIZE,
    {{"color", "black"}, {"zorder", "100"}}
  );

  if (invalid_sectors.size() > 0) {
    plt::arrow(0.0, 0.0, (double) o_hat(0), (double) o_hat(1), LITE_GRAY, LITE_GRAY, 0.1, 0.1);
    plt::annotate(R"($\hat{\mathbf{o}}$)", o_hat(0) + 0.1, o_hat(1) + 0.1);
  }

  CircleSector largest_sector = *max_element(
        valid_sectors.begin(),
        valid_sectors.end(),
        CircleSector::cmp
  );
  eig::Vector2d tmp(
    cos(largest_sector.get_angle_bisector()),
    sin(largest_sector.get_angle_bisector())
  );

  plt::plot(
    {0, 1}, {0, 0}, {{"color", LITE_GRAY}, {"linestyle", "--"}}
  );
  vector<double> bisector_ang_x;
  vector<double> bisector_ang_y;
  double max_ang = largest_sector.get_angle_bisector();
  for (double theta = 0; theta <= max_ang; theta += 0.01) {
    bisector_ang_x.push_back(0.2*cos(theta));
    bisector_ang_y.push_back(0.2*sin(theta));
  }
  plt::plot(bisector_ang_x, bisector_ang_y, {{"color", "black"}});
  plt::annotate(
    R"($\theta$)",
    0.2*cos(max_ang / 2.0) + 0.05,
    0.2*sin(max_ang / 2.0) + 0.05
  );

  plot_line_segment(eig::Vector2d::Zero(), tmp, {{"color", "black"}});

  plt::title(R"(Sectors for agent $\nu_{)" + to_string(beacon_id) + R"(}$)");
  plt::axis("equal");
  plt::show();
}
