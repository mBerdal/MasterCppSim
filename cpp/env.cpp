#include "include/env.h"
namespace eig = Eigen;

Env::Env(const std::string & name) : walls(std::vector<Line>()), name(name), bounding_box() {}

Env::Env(const std::vector<Line> & walls, const std::string & name) : walls(walls), name(name) {
    size_t n_walls = walls.size();
    double* x_mins = new double[n_walls];
    double* y_mins = new double[n_walls];
    double* x_maxs = new double[n_walls];
    double* y_maxs = new double[n_walls];
    for (int i = 0; i < n_walls; i++) {
        x_mins[i] = walls[i].get_bounding_box().get_min_x();
        y_mins[i] = walls[i].get_bounding_box().get_min_y();
        x_maxs[i] = walls[i].get_bounding_box().get_max_x();
        y_maxs[i] = walls[i].get_bounding_box().get_max_y();
    }

    bounding_box = BoundingBox(
        *std::min_element(x_mins, x_mins + n_walls),
        *std::min_element(y_mins, y_mins + n_walls),
        *std::max_element(x_maxs, x_maxs + n_walls),
        *std::max_element(y_maxs, y_maxs + n_walls)
    );
}

Env::Env(const std::list<eig::Matrix<double, 2, eig::Dynamic>> & obstacles, const std::string & name) {
    std::vector<Line> tmp;
    for (eig::Matrix<double, 2, eig::Dynamic> const &obs : obstacles) {
        tmp.emplace_back(obs.col(0), obs.col(obs.cols() - 1));
        for (int col_idx = 1; col_idx < obs.cols(); col_idx++) {
            tmp.emplace_back(obs.col(col_idx), obs.col(col_idx - 1));
        }
    }
    Env(tmp, name);
}


Env Env::ten_by_ten = Env(
    {
        Line(-eig::Vector2d::Ones(), -eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9),
        Line(-eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 9, eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9),
        Line(eig::Vector2d::UnitY() * 9 + eig::Vector2d::UnitX() * 9, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9),
        Line(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9, -eig::Vector2d::Ones())
    },
    "ten_by_ten"
);

Env Env::fifty_by_fifty = Env(
    {
        Line(-eig::Vector2d::Ones(), -eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 49),
        Line(-eig::Vector2d::UnitY() + eig::Vector2d::UnitX() * 49, eig::Vector2d::UnitY() * 49 + eig::Vector2d::UnitX() * 49),
        Line(eig::Vector2d::UnitY() * 49 + eig::Vector2d::UnitX() * 49, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 49),
        Line(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 49, -eig::Vector2d::Ones())
    },
    "fifty_by_fifty"
);

Env Env::stripa_short = Env(
    {
        Line(-eig::Vector2d::UnitX() - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 20 - eig::Vector2d::UnitY()),
        Line(eig::Vector2d::UnitX() * 20 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 2),
        Line(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 2, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 2),
        Line(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 2, eig::Vector2d::UnitX() * 22 - eig::Vector2d::UnitY()),
        Line(eig::Vector2d::UnitX() * 22 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 30 - eig::Vector2d::UnitY()),

        Line(eig::Vector2d::UnitX() * 40 - eig::Vector2d::UnitY(), eig::Vector2d::UnitX() * 40 + eig::Vector2d::UnitY() * 9),

        Line(eig::Vector2d::UnitX() * 40 + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 9),
        Line(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 6),
        Line(eig::Vector2d::UnitX() * 22 + eig::Vector2d::UnitY() * 6, eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 6),
        Line(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 6, eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 9),
        Line(eig::Vector2d::UnitX() * 20 + eig::Vector2d::UnitY() * 9, -eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9),

        Line(-eig::Vector2d::UnitX() + eig::Vector2d::UnitY() * 9, -eig::Vector2d::UnitX() - eig::Vector2d::UnitY())
    },
    "stripa_short"
);

Env Env::stripa = Env(
    {
        Line(eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY()*(-1)),
        Line(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() *(-1 + 3)),
        Line(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3)),
        Line(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY()*(-1)),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2)) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1)),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() *(-1 + 3)),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3)),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() *(-1 + 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY()*(-1)),
        Line(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2)) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1)),
        
        Line(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY()*(-1), eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9),

        Line(eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (((-1 + 20 + 2) + 20 + 2)) + eig::Vector2d::UnitY() * 9),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3)),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * (9 - 3)),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9),
        Line(eig::Vector2d::UnitX() * ((-1 + 20 + 2) + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * ((-1 + 20 + 2)) + eig::Vector2d::UnitY() * 9),
        Line(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3)),
        Line(eig::Vector2d::UnitX() * (-1 + 20 + 2) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * (9 - 3)),
        Line(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * (9 - 3), eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * 9),
        Line(eig::Vector2d::UnitX() * (-1 + 20) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY() * 9),

        Line(eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY() * 9, eig::Vector2d::UnitX() * (-1) + eig::Vector2d::UnitY()*(-1))
    },
    "stripa"
);
