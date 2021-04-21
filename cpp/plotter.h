#ifndef PLOTTER_H
#define PLOTTER_H

#include "matplotlib-cpp/matplotlibcpp.h"
#include "sim.h"
#include "env.h"

using namespace std;

#define FIGURES_DIR "../../figures/"

struct interpolation_color_t {
  int base_r, base_g, base_b, end_r, end_g, end_b;
};

#define DAT_GRADIENT interpolation_color_t{0, 199, 199, 230, 0, 145}

#define BLUE "#5865b8"
#define GREEN "#3d8045"
#define DARK_GRAY "#8A8A8A"
#define LITE_GRAY "#CDCDCD" 
#define RED "#d12a2a"
#define ORANGE "#ff9d00"
#define PURPLE "#c4169c"

#define SCATTER_DOT_SIZE 40.0

void plot_config(Simulator simulator, string run_name = "");
void plot_single_beacon_traj(Simulator simulator, int beacon_id, bool show=false, bool add_legend=false, string run_name="");
void plot_agent_force_vs_time(Simulator simulator, int agent_id, string run_name = "");
void plot_agent_force_vs_dist(Simulator simulator, int agent_id, string run_name = "");
void plot_agent_neigh_traj(Simulator simulator, int agent_id, string run_name = "");
void plot_Xi_model();

#endif