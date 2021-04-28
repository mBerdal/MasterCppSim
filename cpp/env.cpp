#include "env.h"
namespace eig = Eigen;

Env::Env(std::list<eig::Matrix<double, 2, eig::Dynamic>> obstacles) {
    for (eig::Matrix<double, 2, Eigen::Dynamic> const &obs : obstacles) {
        walls.push_back(Wall(obs.col(0), obs.col(obs.cols() - 1)));
        for (int col_idx = 1; col_idx < obs.cols(); col_idx++) {
            walls.push_back(Wall(obs.col(col_idx), obs.col(col_idx - 1)));
        }
    }
}

Env Env::ten_by_ten = Env({
    Wall(-eig::Vector2d::Ones(), -eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9),
    Wall(-eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9, eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9),
    Wall(eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9),
    Wall(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9, -eig::Vector2d::Ones())
});