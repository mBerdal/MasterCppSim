#ifndef PLOTTER_H
#define PLOTTER_H

#include "matplotlib-cpp/matplotlibcpp.h"
#include "sim.h"
#include "env.h"

using namespace std;

#define FIGURES_DIR "../../figures/"

void plot_config(Simulator simulator, string run_name = "");
void plot_agent_force_vs_time(Simulator simulator, int agent_id, string run_name = "");
void plot_agent_force_vs_dist(Simulator simulator, int agent_id, string run_name = "");
void plot_Xi_model();

#endif