#ifndef PLOTTER_H
#define PLOTTER_H

#include "sim.h"

void plot_environment(Simulator simulator, bool show = false);
void plot_config(Simulator simulator, std::string run_name = "");
void plot_single_beacon_traj(Simulator simulator, int beacon_id, bool show=false, bool add_legend=false, std::string run_name="");
void plot_agent_force_vs_time(Simulator simulator, int agent_id, std::string run_name = "");
void plot_agent_force_vs_dist(Simulator simulator, int agent_id, std::string run_name = "");
void plot_agent_neigh_traj(Simulator simulator, int agent_id, std::string run_name = "");
void plot_uniformity_traj(Simulator simulator, std::string run_name = "");
void plot_Xi_model();
void plot_sectors(int beacon_id, std::vector<CircleSector> valid_sectors, std::vector<CircleSector> invalid_sectors, Eigen::Vector2d o_hat);

#endif