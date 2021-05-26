#ifndef PLOTTER_H
#define PLOTTER_H

#include "sim.h"

void plot_environment(const Simulator & simulator, bool show = false);
void plot_config(const Simulator & simulator, bool show = false, std::string run_name = "");
void plot_single_beacon_traj(const Simulator & simulator, int beacon_id, bool show=false, bool add_legend=false, std::string run_name="");
void plot_uniformity_traj(const Simulator & simulator, bool show=false, std::string run_name = "");
void plot_agent_force_vs_time(const Simulator & simulator, int agent_id, std::string run_name = "");
void plot_agent_force_vs_dist(const Simulator & simulator, int agent_id, std::string run_name = "");
void plot_agent_neigh_traj(Simulator simulator, int agent_id, std::string run_name = "");
void plot_Xi_model();
void plot_sectors(int beacon_id, std::vector<CircleSector> valid_sectors, std::vector<CircleSector> invalid_sectors, Eigen::Vector2d o_hat);

#endif