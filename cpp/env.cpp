#include "include/env.h"
namespace eig = Eigen;

Env::Env(std::list<eig::Matrix<double, 2, eig::Dynamic>> obstacles) {
    for (eig::Matrix<double, 2, Eigen::Dynamic> const &obs : obstacles) {
        walls.emplace_back(obs.col(0), obs.col(obs.cols() - 1));
        for (int col_idx = 1; col_idx < obs.cols(); col_idx++) {
            walls.emplace_back(obs.col(col_idx), obs.col(col_idx - 1));
        }
    }
}

Env Env::ten_by_ten = Env({
    Wall(-eig::Vector2d::Ones(), -eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9),
    Wall(-eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9, eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9),
    Wall(eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9),
    Wall(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9, -eig::Vector2d::Ones())
});

Env Env::stripa_short = Env({
    Wall(-eig::Vector2d::UnitX() - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 20 - eig::Vector2d::UnitY()),
    Wall(eig::Vector2d::UnitX() * 20 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 2),
    Wall(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 2, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 2),
    Wall(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 2, eig::Vector2d::UnitX() * 22 - eig::Vector2d::UnitY()),
    Wall(eig::Vector2d::UnitX() * 22 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 30 - eig::Vector2d::UnitY()),

    Wall(eig::Vector2d::UnitX() * 40 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 40 + eig::Vector2d::UnitY() * 9),

    Wall(eig::Vector2d::UnitX() * 40 + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 9),
    Wall(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 6),
    Wall(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 6, eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 6),
    Wall(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 6, eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 9),
    Wall(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 9, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9),

    Wall(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9, -eig::Vector2d::UnitX() - eig::Vector2d::UnitY())
});

Env Env::stripa = Env({
    Wall(eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY()*(-1)),
    Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() *(-1 + 3)),
    Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3)),
    Wall(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY()*(-1)),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2)) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1)),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() *(-1 + 3)),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3)),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY()*(-1)),
    Wall(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2)) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1)),
    
    Wall(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9),

    Wall(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2)) + eig::Vector2d::UnitY() * 9),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3)),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * (9 - 3)),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9),
    Wall(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * ((-1 + 20 + 2)) + eig::Vector2d::UnitY() * 9),
    Wall(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3)),
    Wall(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * (9 - 3)),
    Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * 9),
    Wall(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY() * 9),

    Wall(eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY()*(-1))
});
