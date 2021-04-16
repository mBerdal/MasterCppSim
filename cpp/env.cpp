#include "env.h"

Env::Env(std::list<Matrix<double, 2, Eigen::Dynamic>> obstacles) {
    for (Matrix<double, 2, Eigen::Dynamic> const &obs : obstacles) {
        walls.push_back(Wall(obs.col(0), obs.col(obs.cols() - 1)));
        for (int col_idx = 1; col_idx < obs.cols(); col_idx++) {
            walls.push_back(Wall(obs.col(col_idx), obs.col(col_idx - 1)));
        }
    }
}